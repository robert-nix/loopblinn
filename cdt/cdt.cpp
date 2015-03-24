#include <stdint.h>
#include <stdlib.h>
#include <float.h>
#include <stdio.h>

typedef float f32;
typedef int32_t s32;
typedef int8_t s8;

template <typename T> struct Vec {
	T *data = 0;
	s32 size = 0;
	s32 capacity = 0;

	~Vec() {
		if (data) {
			free(data);
		}
	}

	void push_back(T x) {
		if (size >= capacity) {
			s32 cap = capacity;
			if (cap == 0) {
				cap = sizeof(T) >= 16 ? 4 : 64 / sizeof(T);
			} else {
				cap = (cap * 3) / 2;
			}
			resize(cap);
		}
		data[size] = x;
		size++;
	}

	void reserve(s32 cap) {
		if (cap > capacity) {
			resize(cap);
		}
	}

	void resize(s32 cap) {
		data = (T *)realloc(data, sizeof(T) * cap);
		capacity = cap;
	}

	T &operator[](s32 idx) { return data[idx]; }
	const T &operator[](s32 idx) const { return data[idx]; }
};

struct Triangulation {
	f32 *Verts;
	s32 *Edges;
	s32 *Triangles;
	s8 *fixed;
	s32 *newTris;
	s32 *newEdges;
	s32 *checkTris;
	s32 VertI, EdgeI, TriangleI;
	s32 newTriI, newEdgeI, checkTriI;

	s32 AddPoint(f32 x, f32 y);
	void AddEdge(s32 indexA, s32 indexB);
	void retriangulate(Vec<s32> &ringEdges, Vec<s32> &vertIs, s32 edgeIs[2]);
	void getSharedQuad(s32 quad[4], s32 triA, s32 triB);
};

extern "C" s32 triangulate(f32 left, f32 right, f32 bottom, f32 top,
                           s32 nPoints, f32 *points, s32 nEdges, s32 *edges,
                           f32 *verts, s32 *srcToDstIs, s32 *triangles) {
	Triangulation t = {0};
	t.Verts = verts;
	t.Triangles = triangles;
	s32 rPoints = nPoints + 4;
	s32 rTris = 2 * rPoints - 6;
	s32 rEdges = 3 * rPoints - 7;
	t.newTris = (s32 *)malloc(4 * 3 * rTris);
	t.Edges = (s32 *)malloc(4 * 2 * rEdges);
	t.newEdges = (s32 *)malloc(4 * 2 * rEdges);
	t.fixed = (s8 *)malloc(rEdges);
	t.checkTris = (s32 *)malloc(4 * rTris);

	t.Verts[0] = left;
	t.Verts[1] = bottom;
	t.Verts[2] = left;
	t.Verts[3] = top;
	t.Verts[4] = right;
	t.Verts[5] = bottom;
	t.Verts[6] = right;
	t.Verts[7] = top;
	t.VertI = 8;
	t.Edges[0] = 0;
	t.Edges[1] = 2;
	t.Edges[2] = 0;
	t.Edges[3] = 4;
	t.Edges[4] = 2;
	t.Edges[5] = 4;
	t.Edges[6] = 2;
	t.Edges[7] = 6;
	t.Edges[8] = 4;
	t.Edges[9] = 6;
	t.EdgeI = 10;
	t.Triangles[0] = 0;
	t.Triangles[1] = 2;
	t.Triangles[2] = 4;
	t.Triangles[3] = 4;
	t.Triangles[4] = 2;
	t.Triangles[5] = 6;
	t.TriangleI = 6;
	t.fixed[0] = true;
	t.fixed[1] = true;
	t.fixed[2] = false;
	t.fixed[3] = true;
	t.fixed[4] = true;

	for (s32 i = 0; i < nPoints; i++) {
		s32 ptI = t.AddPoint(points[2 * i], points[2 * i + 1]);
		if (ptI < 0) {
			return ptI;
		}
		srcToDstIs[i] = ptI;
	}

	for (s32 i = 0; i < nEdges; i++) {
		t.AddEdge(srcToDstIs[edges[2 * i]], srcToDstIs[edges[2 * i + 1]]);
	}

	free(t.newTris);
	free(t.Edges);
	free(t.newEdges);
	free(t.fixed);
	free(t.checkTris);

	return t.VertI;
}

struct indexedVert {
	s32 i;
	f32 f;
};

int indexedVertCompare(const void *a, const void *b) {
	auto iva = (const indexedVert *)a;
	auto ivb = (const indexedVert *)b;
	return iva->f < ivb->f ? -1 : iva->f > ivb->f;
}

// px, py, ax, ay, bx, by, cx, cy
void getBarycentric(f32 bary[2], f32 ps[8]) {
	f32 v0x = ps[6] - ps[2];
	f32 v0y = ps[7] - ps[3];
	f32 v1x = ps[4] - ps[2];
	f32 v1y = ps[5] - ps[3];
	f32 v2x = ps[0] - ps[2];
	f32 v2y = ps[1] - ps[3];
	f32 dot00 = v0x * v0x + v0y * v0y;
	f32 dot01 = v0x * v1x + v0y * v1y;
	f32 dot02 = v0x * v2x + v0y * v2y;
	f32 dot11 = v1x * v1x + v1y * v1y;
	f32 dot12 = v1x * v2x + v1y * v2y;
	f32 norm = dot00 * dot11 - dot01 * dot01;
	bary[0] = (dot11 * dot02 - dot01 * dot12) / norm;
	bary[1] = (dot00 * dot12 - dot01 * dot02) / norm;
}

s8 pointInTriangle(f32 ps[8]) {
	f32 bary[2];
	getBarycentric(bary, ps);
	return bary[0] >= -4e-6f && bary[1] >= -4e-6f &&
	       bary[0] + bary[1] < 1.0000076f;
}

s8 pointInAngle(f32 ps[8]) {
	f32 bary[2];
	getBarycentric(bary, ps);
	return bary[0] >= -4e-6f && bary[1] >= -4e-6f;
}

f32 mat4Det(f32 m[16]) {
	return m[0] * m[5] * m[10] * m[15] - m[0] * m[5] * m[11] * m[14] -
	       m[0] * m[6] * m[9] * m[15] + m[0] * m[6] * m[11] * m[13] +
	       m[0] * m[7] * m[9] * m[14] - m[0] * m[7] * m[10] * m[13] -
	       m[1] * m[4] * m[10] * m[15] + m[1] * m[4] * m[11] * m[14] +
	       m[1] * m[6] * m[8] * m[15] - m[1] * m[6] * m[11] * m[12] -
	       m[1] * m[7] * m[8] * m[14] + m[1] * m[7] * m[10] * m[12] +
	       m[2] * m[4] * m[9] * m[15] - m[2] * m[4] * m[11] * m[13] -
	       m[2] * m[5] * m[8] * m[15] + m[2] * m[5] * m[11] * m[12] +
	       m[2] * m[7] * m[8] * m[13] - m[2] * m[7] * m[9] * m[12] -
	       m[3] * m[4] * m[9] * m[14] + m[3] * m[4] * m[10] * m[13] +
	       m[3] * m[5] * m[8] * m[14] - m[3] * m[5] * m[10] * m[12] -
	       m[3] * m[6] * m[8] * m[13] + m[3] * m[6] * m[9] * m[12];
}

s32 Triangulation::AddPoint(f32 x, f32 y) {
	s8 duplicate = false;
	s8 found = false;
	s32 parentTriIs[2] = {-1, -1};
	s32 parentTriIsIdx = 0;
	f32 minVertDist = FLT_MAX;
	s32 minVertI = -1;
	for (s32 i = 0; i < VertI; i += 2) {
		f32 vx = Verts[i] - x;
		f32 vy = Verts[i + 1] - y;
		f32 vertDist = vx * vx + vy * vy;
		if (vertDist < minVertDist) {
			minVertI = i;
			minVertDist = vertDist;
		}
	}
#define findTriForVert(vertI)                                                  \
	for (s32 i = 0; i < TriangleI; i += 3) {                                   \
		if (Triangles[i] != (vertI) && Triangles[i + 1] != (vertI) &&          \
		    Triangles[i + 2] != (vertI)) {                                     \
			continue;                                                          \
		}                                                                      \
		f32 ps[8] = {x, y, Verts[Triangles[i]], Verts[Triangles[i] + 1],       \
		             Verts[Triangles[i + 1]], Verts[Triangles[i + 1] + 1],     \
		             Verts[Triangles[i + 2]], Verts[Triangles[i + 2] + 1]};    \
		if (pointInTriangle(ps)) {                                             \
			found = true;                                                      \
			if (parentTriIsIdx < 2) {                                          \
				parentTriIs[parentTriIsIdx] = i;                               \
				parentTriIsIdx++;                                              \
			} else {                                                           \
				duplicate = true;                                              \
				break;                                                         \
			}                                                                  \
		}                                                                      \
	}
	findTriForVert(minVertI);
	if (!found) {
		auto sortedVerts = (indexedVert *)malloc(8 * VertI);
		for (s32 i = 0; i < VertI; i += 2) {
			f32 vx = Verts[i] - x;
			f32 vy = Verts[i + 1] - y;
			f32 vertDist = vx * vx + vy * vy;
			sortedVerts[i].i = i;
			sortedVerts[i].f = vertDist;
		}
		qsort(sortedVerts, VertI, 8, indexedVertCompare);
		for (s32 j = 0; j < VertI; j++) {
			findTriForVert(sortedVerts[j].i);
			if (found) {
				break;
			}
		}
		free(sortedVerts);
	}
	if (!found) {
		// out of bounds
		return -1;
	}
	if (duplicate) {
		for (s32 dupI = 0; dupI < VertI; dupI += 2) {
			f32 vx = Verts[dupI] - x;
			f32 vy = Verts[dupI + 1] - y;
			if (vx * vx + vy * vy < 1e-8f) {
				return dupI;
			}
		}
		return -1;
	}

	s32 ptI = VertI;
	Verts[ptI] = x;
	Verts[ptI + 1] = y;
	VertI += 2;
	checkTriI = 0;
	if (parentTriIs[1] != -1) {
		s32 quad[4];
		getSharedQuad(quad, parentTriIs[0], parentTriIs[1]);
		s32 edge[2] = {quad[1], quad[3]};
		if (quad[1] > quad[3]) {
			edge[0] = quad[3];
			edge[1] = quad[1];
		}
		s32 oldEdgeI = -1;
		for (s32 i = 0; i < EdgeI; i += 2) {
			if (Edges[i] == edge[0] && Edges[i + 1] == edge[1]) {
				oldEdgeI = i;
				break;
			}
		}
		// generate new tris and edges
		// 0:
		Triangles[parentTriIs[0]] = quad[0];
		Triangles[parentTriIs[0] + 1] = quad[1];
		Triangles[parentTriIs[0] + 2] = ptI;
		// 1:
		Triangles[parentTriIs[1]] = quad[1];
		Triangles[parentTriIs[1] + 1] = quad[2];
		Triangles[parentTriIs[1] + 2] = ptI;
		// our edge sortedness is guaranteed here because ptI is the
		// largest index
		Edges[oldEdgeI] = quad[1];
		Edges[oldEdgeI + 1] = ptI;

		Triangles[TriangleI + 0] = quad[2];
		Triangles[TriangleI + 1] = quad[3];
		Triangles[TriangleI + 2] = ptI;
		Triangles[TriangleI + 3] = quad[3];
		Triangles[TriangleI + 4] = quad[0];
		Triangles[TriangleI + 5] = ptI;
		Edges[EdgeI + 0] = quad[2];
		Edges[EdgeI + 1] = ptI;
		Edges[EdgeI + 2] = quad[3];
		Edges[EdgeI + 3] = ptI;
		Edges[EdgeI + 4] = quad[0];
		Edges[EdgeI + 5] = ptI;
		s32 fixedI = EdgeI >> 1;
		fixed[fixedI + 0] = false;
		fixed[fixedI + 1] = false;
		fixed[fixedI + 2] = false;
		checkTris[checkTriI + 0] = parentTriIs[0];
		checkTris[checkTriI + 1] = parentTriIs[1];
		checkTris[checkTriI + 2] = TriangleI;
		checkTris[checkTriI + 3] = TriangleI + 3;
		TriangleI += 6;
		EdgeI += 6;
		checkTriI += 4;
	} else {
		s32 triI = parentTriIs[0];
		s32 triVs[3] = {Triangles[triI], Triangles[triI + 1],
		                Triangles[triI + 2]};
		Triangles[triI] = triVs[0];
		Triangles[triI + 1] = triVs[1];
		Triangles[triI + 2] = ptI;

		Triangles[TriangleI + 0] = triVs[1];
		Triangles[TriangleI + 1] = triVs[2];
		Triangles[TriangleI + 2] = ptI;
		Triangles[TriangleI + 3] = triVs[2];
		Triangles[TriangleI + 4] = triVs[0];
		Triangles[TriangleI + 5] = ptI;
		Edges[EdgeI + 0] = triVs[1];
		Edges[EdgeI + 1] = ptI;
		Edges[EdgeI + 2] = triVs[2];
		Edges[EdgeI + 3] = ptI;
		Edges[EdgeI + 4] = triVs[0];
		Edges[EdgeI + 5] = ptI;
		s32 fixedI = EdgeI >> 1;
		fixed[fixedI + 0] = false;
		fixed[fixedI + 1] = false;
		fixed[fixedI + 2] = false;
		checkTris[checkTriI + 0] = triI;
		checkTris[checkTriI + 1] = TriangleI;
		checkTris[checkTriI + 2] = TriangleI + 3;
		TriangleI += 6;
		EdgeI += 6;
		checkTriI += 3;
	}
	while (checkTriI > 0) {
		s32 triI = checkTris[0];
		s32 triV[3] = {Triangles[triI], Triangles[triI + 1],
		               Triangles[triI + 2]};
		for (s32 i = 0; i < 3; i++) {
			s32 edge[2] = {triV[(i + 1) % 3], triV[i]};
			s32 sortedEdge[2] = {edge[0], edge[1]};
			if (edge[0] > edge[1]) {
				sortedEdge[0] = edge[1];
				sortedEdge[1] = edge[0];
			}
			s32 edgeI = 0;
			for (; edgeI < EdgeI; edgeI += 2) {
				if (Edges[edgeI] == sortedEdge[0] &&
				    Edges[edgeI + 1] == sortedEdge[1]) {
					break;
				}
			}
			if (fixed[edgeI >> 1]) {
				continue;
			}
			found = false;
			s32 otherTriI = -1;
			for (s32 n = 0; n < TriangleI; n += 3) {
				if (n == triI) {
					continue;
				}
				if (Triangles[n] == edge[0] && Triangles[n + 1] == edge[1]) {
					found = true;
					otherTriI = n;
					break;
				}
				if (Triangles[n + 1] == edge[0] &&
				    Triangles[n + 2] == edge[1]) {
					found = true;
					otherTriI = n;
					break;
				}
				if (Triangles[n + 2] == edge[0] && Triangles[n] == edge[1]) {
					found = true;
					otherTriI = n;
					break;
				}
			}
			if (!found) {
				continue;
			}
			s32 quad[4];
			getSharedQuad(quad, triI, otherTriI);
			f32 sign;
			{
				f32 ax, bx, cx, dx, ay, by, cy, dy;
				ax = Verts[quad[0]];
				ay = Verts[quad[0] + 1];
				bx = Verts[quad[1]];
				by = Verts[quad[1] + 1];
				cx = Verts[quad[2]];
				cy = Verts[quad[2] + 1];
				dx = Verts[quad[3]];
				dy = Verts[quad[3] + 1];
				f32 mat[16] = {//--------------------------
				               dx, dy, dx * dx + dy * dy, 1,
				               //--------------------------
				               cx, cy, cx * cx + cy * cy, 1,
				               //--------------------------
				               bx, by, bx * bx + by * by, 1,
				               //--------------------------
				               ax, ay, ax * ax + ay * ay, 1};
				sign = mat4Det(mat);
			}
			if (sign > 1e-7f) {
				Triangles[triI] = quad[0];
				Triangles[triI + 1] = quad[1];
				Triangles[triI + 2] = quad[2];
				Triangles[otherTriI] = quad[0];
				Triangles[otherTriI + 1] = quad[2];
				Triangles[otherTriI + 2] = quad[3];
				s32 newEdge[2] = {quad[0], quad[2]};
				if (newEdge[0] > newEdge[1]) {
					newEdge[0] = quad[2];
					newEdge[1] = quad[0];
				}
				Edges[edgeI] = newEdge[0];
				Edges[edgeI + 1] = newEdge[1];
				checkTris[checkTriI + 0] = triI;
				checkTris[checkTriI + 1] = otherTriI;
				checkTriI += 2;
				break;
			}
		}
		checkTris[0] = checkTris[checkTriI - 1];
		checkTriI--;
	}

	return ptI;
}

void Triangulation::AddEdge(s32 indexA, s32 indexB) {
	if (indexA == indexB) {
		return;
	}
	s32 edge[2] = {indexA, indexB};
	if (edge[0] > edge[1]) {
		edge[0] = indexB;
		edge[1] = indexA;
	}
	s8 edgeExists = false;
	for (s32 i = 0; i < EdgeI; i += 2) {
		if (Edges[i] == edge[0] && Edges[i + 1] == edge[1]) {
			edgeExists = true;
			fixed[i >> 1] = true;
			break;
		}
	}
	if (edgeExists) {
		return;
	}
	s32 crossedTri[3] = {-1, -1, -1};
	s32 crossedTriI = -1;
	for (s32 i = 0; i < TriangleI; i += 3) {
		if (Triangles[i] == edge[0]) {
			f32 ps[8] = {Verts[edge[1]], Verts[edge[1] + 1],
			             Verts[Triangles[i]], Verts[Triangles[i] + 1],
			             Verts[Triangles[i + 1]], Verts[Triangles[i + 1] + 1],
			             Verts[Triangles[i + 2]], Verts[Triangles[i + 2] + 1]};
			if (pointInAngle(ps)) {
				crossedTri[0] = Triangles[i];
				crossedTri[1] = Triangles[i + 1];
				crossedTri[2] = Triangles[i + 2];
				crossedTriI = i;
				break;
			}
		}
		if (Triangles[i + 1] == edge[0]) {
			f32 ps[8] = {Verts[edge[1]], Verts[edge[1] + 1],
			             Verts[Triangles[i + 1]], Verts[Triangles[i + 1] + 1],
			             Verts[Triangles[i + 2]], Verts[Triangles[i + 2] + 1],
			             Verts[Triangles[i]], Verts[Triangles[i] + 1]};
			if (pointInAngle(ps)) {
				crossedTri[0] = Triangles[i + 1];
				crossedTri[1] = Triangles[i + 2];
				crossedTri[2] = Triangles[i];
				crossedTriI = i;
				break;
			}
		}
		if (Triangles[i + 2] == edge[0]) {
			f32 ps[8] = {Verts[edge[1]], Verts[edge[1] + 1],
			             Verts[Triangles[i + 2]], Verts[Triangles[i + 2] + 1],
			             Verts[Triangles[i]], Verts[Triangles[i] + 1],
			             Verts[Triangles[i + 1]], Verts[Triangles[i + 1] + 1]};
			if (pointInAngle(ps)) {
				crossedTri[0] = Triangles[i + 2];
				crossedTri[1] = Triangles[i];
				crossedTri[2] = Triangles[i + 1];
				crossedTriI = i;
				break;
			}
		}
	}
	if (crossedTriI == -1) {
		printf("crossedTri wasn't found (%d, %d)\n", edge[0], edge[1]);
		return;
	}
	f32 ptA[2] = {Verts[edge[0]], Verts[edge[0] + 1]};
	f32 ptB[2] = {Verts[edge[1]], Verts[edge[1] + 1]};
	Vec<s32> ptsU{};
	ptsU.push_back(crossedTri[1]);
	Vec<s32> ptsL{};
	ptsL.push_back(crossedTri[2]);
	Vec<s32> deadTriIs{};
	deadTriIs.push_back(crossedTriI);
	Vec<s32> deadEdges{};
	newTriI = 0;
	newEdgeI = 0;
	Vec<s32> ringEdges{};
	ringEdges.push_back(crossedTri[0]);
	ringEdges.push_back(crossedTri[1]);
	ringEdges.push_back(crossedTri[0]);
	ringEdges.push_back(crossedTri[2]);
	for (;;) {
		s32 otherTriI = -1;
		s32 otherVertI = -1;
		for (s32 n = 0; n < TriangleI; n += 3) {
			if (n == crossedTriI) {
				continue;
			}
			if (Triangles[n] == crossedTri[2] &&
			    Triangles[n + 1] == crossedTri[1]) {
				otherTriI = n;
				otherVertI = Triangles[n + 2];
				break;
			}
			if (Triangles[n + 1] == crossedTri[2] &&
			    Triangles[n + 2] == crossedTri[1]) {
				otherTriI = n;
				otherVertI = Triangles[n];
				break;
			}
			if (Triangles[n + 2] == crossedTri[2] &&
			    Triangles[n] == crossedTri[1]) {
				otherTriI = n;
				otherVertI = Triangles[n + 1];
				break;
			}
		}
		deadTriIs.push_back(otherTriI);
		deadEdges.push_back(crossedTri[1]);
		deadEdges.push_back(crossedTri[2]);
		if (deadTriIs.size > 10000) {
			return;
		}
		if (otherVertI == edge[1]) {
			ringEdges.push_back(crossedTri[1]);
			ringEdges.push_back(otherVertI);
			ringEdges.push_back(crossedTri[2]);
			ringEdges.push_back(otherVertI);
			break;
		}
		f32 oppositePt[2] = {Verts[otherVertI], Verts[otherVertI + 1]};
		f32 ptSide = (ptB[0] - ptA[0]) * (oppositePt[1] - ptA[1]) -
		             (ptB[1] - ptA[1]) * (oppositePt[0] - ptA[0]);
		if (ptSide > 1e-12f) {
			ptsU.push_back(otherVertI);
			ringEdges.push_back(crossedTri[1]);
			ringEdges.push_back(otherVertI);
			crossedTri[0] = crossedTri[1];
			crossedTri[1] = otherVertI;
			crossedTriI = otherTriI;
		} else if (ptSide < -1e-12f) {
			ptsL.push_back(otherVertI);
			ringEdges.push_back(crossedTri[2]);
			ringEdges.push_back(otherVertI);
			crossedTri[0] = crossedTri[2];
			crossedTri[2] = otherVertI;
			crossedTriI = otherTriI;
		} else {
			AddEdge(otherVertI, edge[1]);
			edge[1] = otherVertI;
			break;
		}
	}
	retriangulate(ringEdges, ptsU, edge);
	s32 tmp = edge[0];
	edge[0] = edge[1];
	edge[1] = tmp;
	retriangulate(ringEdges, ptsL, edge);
	if (edge[0] > edge[1]) {
		s32 tmp = edge[0];
		edge[0] = edge[1];
		edge[1] = tmp;
	}
	newEdgeI -= 2;
	s32 nDeadTriIs = deadTriIs.size;
	for (s32 i = 0; i < nDeadTriIs; i++) {
		Triangles[deadTriIs[i]] = newTris[3 * i];
		Triangles[deadTriIs[i] + 1] = newTris[3 * i + 1];
		Triangles[deadTriIs[i] + 2] = newTris[3 * i + 2];
	}
	s32 nDeadEdges = deadEdges.size;
	for (s32 i = 0; i < nDeadEdges; i += 2) {
		s32 deadEdge[2] = {deadEdges[i], deadEdges[i + 1]};
		if (deadEdges[i] > deadEdges[i + 1]) {
			deadEdge[0] = deadEdges[i + 1];
			deadEdge[1] = deadEdges[i];
		}
		s32 newEdge[2] = {newEdges[i], newEdges[i + 1]};
		if (newEdges[i] > newEdges[i + 1]) {
			newEdge[0] = newEdges[i + 1];
			newEdge[1] = newEdges[i];
		}
		s8 last = i == nDeadEdges - 2;
		for (s32 j = 0; j < EdgeI; j += 2) {
			if (deadEdge[0] == Edges[j] && deadEdge[1] == Edges[j + 1]) {
				Edges[j] = newEdge[0];
				Edges[j + 1] = newEdge[1];
				if (last) {
					fixed[j >> 1] = true;
				}
				break;
			}
		}
	}
}

void Triangulation::retriangulate(Vec<s32> &ringEdges, Vec<s32> &vertIs,
                                  s32 edgeIs[2]) {
	s32 cI = -1;
	s32 nVertIs = vertIs.size;
	if (nVertIs > 1) {
		cI = vertIs[0];
		f32 c[2] = {Verts[cI], Verts[cI + 1]};
		f32 a[2] = {Verts[edgeIs[0]], Verts[edgeIs[0] + 1]};
		f32 b[2] = {Verts[edgeIs[1]], Verts[edgeIs[1] + 1]};
		for (s32 i = 1; i < nVertIs; i++) {
			f32 d[2] = {Verts[vertIs[i]], Verts[vertIs[i] + 1]};
			f32 mat[16] = {a[0], a[1], a[0] * a[0] + a[1] * a[1], 1,
			               // -------------------------------------
			               b[0], b[1], b[0] * b[0] + b[1] * b[1], 1,
			               // -------------------------------------
			               c[0], c[1], c[0] * c[0] + c[1] * c[1], 1,
			               // -------------------------------------
			               d[0], d[1], d[0] * d[0] + d[1] * d[1], 1};
			f32 sign = mat4Det(mat);
			if (sign > 0.f) {
				cI = vertIs[i];
				c[0] = Verts[cI];
				c[1] = Verts[cI + 1];
			}
		}
		Vec<s32> leftVertIs{};
		Vec<s32> rightVertIs{};
		s32 ringI = edgeIs[0];
		s32 prevRingI = -1;
		s8 right = false;
		s32 nRingEdges = ringEdges.size;
		for (;;) {
			s8 inRing = false;
			s8 shouldBreak = false;
			s32 testI = -1;
			for (s32 i = 0; i < nRingEdges; i += 2) {
				if (ringEdges[i] == ringI) {
					testI = ringEdges[i + 1];
				}
				if (ringEdges[i + 1] == ringI) {
					testI = ringEdges[i];
				}
				if (testI >= 0 && prevRingI != testI) {
					inRing = false;
					shouldBreak = false;
					for (s32 j = 0; j < nVertIs; j++) {
						if (vertIs[j] == testI) {
							inRing = true;
							shouldBreak = false;
							break;
						}
					}
					if (!inRing && (testI == edgeIs[1] || testI == edgeIs[0])) {
						inRing = true;
						shouldBreak = true;
						break;
					}
					if (inRing) {
						prevRingI = ringI;
						ringI = testI;
						if (ringI == cI) {
							right = true;
						} else if (right) {
							rightVertIs.push_back(ringI);
						} else {
							leftVertIs.push_back(ringI);
						}
						break;
					}
				}
			}
			if (shouldBreak) {
				break;
			}
			if (ringI == prevRingI) {
				return;
			}
		}
		s32 leftEdge[2] = {edgeIs[0], cI};
		s32 rightEdge[2] = {cI, edgeIs[1]};
		retriangulate(ringEdges, leftVertIs, leftEdge);
		retriangulate(ringEdges, rightVertIs, rightEdge);
	}
	if (nVertIs > 0) {
		if (cI == -1) {
			cI = vertIs[0];
		}
		newTris[newTriI + 0] = edgeIs[1];
		newTris[newTriI + 1] = edgeIs[0];
		newTris[newTriI + 2] = cI;
		newEdges[newEdgeI + 0] = edgeIs[0];
		newEdges[newEdgeI + 1] = edgeIs[1];
		newTriI += 3;
		newEdgeI += 2;
	}
}

void Triangulation::getSharedQuad(s32 quad[4], s32 triA, s32 triB) {
	s32 tris[2] = {triA, triB};
	for (s32 n = 0; n < 2; n++) {
		triA = tris[n];
		triB = tris[1 - n];
		for (s32 j = 0; j < 3; j++) {
			s32 m = Triangles[triA + j];
			s8 found = false;
			for (s32 i = 0; i < 3; i++) {
				if (Triangles[triB + i] == m) {
					found = true;
					break;
				}
			}
			if (!found) {
				quad[n * 2] = m;
				quad[n * 2 + 1] = Triangles[triA + ((j + 1) % 3)];
				break;
			}
		}
	}
}

#ifdef CDT_TEST
int main() {
	s32 numVerts = 2 * (67 + 4);
	s32 numTris = 3 * (2 * (67 + 4) - 6);
	f32 *verts = (f32 *)malloc(4 * numVerts);
	s32 *srcToDstIs = (s32 *)malloc(4 * 67);
	s32 *tris = (s32 *)malloc(4 * numTris);

	f32 points[] = {
	    0.8500061,   -0.11500549, 0.79499817,  -0.11500549, 0.79499817,
	    0.7859955,   0.8500061,   0.7859955,   0.6719971,   -0.05000305,
	    0.6150055,   -0.05000305, 0.6150055,   0.3730011,   0.45300293,
	    0.3730011,   0.42199707,  0.30200195,  0.3789978,   0.24349976,
	    0.3789978,   0.24349976,  0.33599854,  0.18499756,  0.28199768,
	    0.14500427,  0.28199768,  0.14500427,  0.23800659,  0.17700195,
	    0.2899933,   0.21699524,  0.32899475,  0.26999664,  0.32899475,
	    0.26999664,  0.36799622,  0.32299805,  0.3939972,   0.38000488,
	    0.3939972,   0.38000488,  0.41999817,  0.43800354,  0.43499756,
	    0.4960022,   0.43499756,  0.4960022,   0.44999695,  0.55400085,
	    0.45500183,  0.6049957,   0.45500183,  0.6049957,   0.30599976,
	    0.6049957,   0.30599976,  0.65400696,  0.5209961,   0.65400696,
	    0.5149994,   0.5930023,   0.50250244,  0.5345001,   0.50250244,
	    0.5345001,   0.4900055,   0.47599792,  0.47200012,  0.42300415,
	    0.47200012,  0.42300415,  0.6150055,   0.42300415,  0.6150055,
	    0.7649994,   0.6719971,   0.7649994,   0.29600525,  0.65400696,
	    0.29100037,  0.576004,    0.27300262,  0.5019989,   0.27300262,
	    0.5019989,   0.25500488,  0.42799377,  0.22399902,  0.3630066,
	    0.22399902,  0.3630066,   0.19299316,  0.29800415,  0.14949799,
	    0.2444992,   0.14949799,  0.2444992,   0.10600281,  0.19099426,
	    0.052001953, 0.15499878,  0.052001953, 0.15499878,  0.009002686,
	    0.19000244,  0.070007324, 0.22999573,  0.11100006,  0.28450012,
	    0.11100006,  0.28450012,  0.1519928,   0.33900452,  0.17799377,
	    0.3959961,   0.17799377,  0.3959961,   0.20300293,  0.45300293,
	    0.21549988,  0.50800323,  0.21549988,  0.50800323,  0.22799683,
	    0.56300354,  0.23199463,  0.6049957,   0.23199463,  0.6049957,
	    0.06199646,  0.6049957,   0.06199646,  0.65400696};

	s32 edges[] = {7,  8,  8,  9,  9,  7,  10, 11, 11, 12, 12, 10, 14, 15, 15,
	               16, 16, 14, 17, 18, 18, 19, 19, 17, 20, 21, 21, 22, 22, 20,
	               23, 24, 24, 25, 25, 23, 29, 30, 30, 31, 31, 29, 32, 33, 33,
	               34, 34, 32, 39, 40, 40, 41, 41, 39, 42, 43, 43, 44, 44, 42,
	               45, 46, 46, 47, 47, 45, 48, 49, 49, 50, 50, 48, 52, 53, 53,
	               54, 54, 52, 55, 56, 56, 57, 57, 55, 58, 59, 59, 60, 60, 58,
	               61, 62, 62, 63, 63, 61, 0,  1,  1,  2,  2,  3,  3,  0,  4,
	               5,  5,  6,  6,  7,  13, 14, 26, 27, 27, 28, 28, 29, 35, 36,
	               36, 37, 37, 38, 38, 4,  51, 52, 64, 65, 65, 66, 66, 39};

	for (s32 i = 0; i < 1000; i++) {
		triangulate(-0.033047f, 0.892056f, -0.160056f, 0.831046f, 67, points,
		            67, edges, verts, srcToDstIs, tris);
	}

	for (s32 i = 0; i < numVerts; i += 2) {
		printf("%d: %.6f %.6f\n", i, verts[i], verts[i + 1]);
	}
	for (s32 i = 0; i < numTris; i += 3) {
		printf("%d: %d %d %d\n", i, tris[i], tris[i + 1], tris[i + 2]);
	}
}
#endif