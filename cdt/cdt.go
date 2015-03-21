/*
Package cdt provides an implementation of 2-dimensional Constrained Delaunay
Triangulation.  The implementation follows the algorithm outlined at
http://www.cescg.org/CESCG-2004/web/Domiter-Vid/index.html, but without any of
the spatial data structures.
*/
package cdt

import (
	"github.com/go-gl/mathgl/mgl32"
)

type Triangulation struct {
	Verts []mgl32.Vec2
	// Edge pairs are always stored with the smaller vert index first
	Edges []int
	// Triangles are always stored with clockwise winding
	Triangles []int
	// This isn't currently enforced, but could be useful for error checking
	fixed []bool
	// Used for edge insertion:
	newTris, newEdges []int
}

// NewTriangulation returns a new Triangulation initialized for performing
// triangulation of points that exist within the region defined by left and
// right on the X axis and by bottom and top on the Y axis.  Points added to
// the triangulation should exist within the boundary defined by left, right,
// bottom, and top, and they should not fall directly on the edges of or outside
// of this rectangle.
func NewTriangulation(left, right, bottom, top float32) *Triangulation {
	result := &Triangulation{}
	// initialize with 2 triangles to form a bounding quad:
	result.Verts = []mgl32.Vec2{
		{left, bottom},
		{left, top},
		{right, bottom},
		{right, top},
	}
	result.Edges = []int{
		0, 1,
		0, 2,
		1, 2,
		1, 3,
		2, 3,
	}
	result.Triangles = []int{
		0, 1, 2,
		2, 1, 3,
	}
	result.fixed = []bool{
		// the border edges are fixed:
		true, true, false, true, true,
	}
	return result
}

// AddPoint inserts the point defined by x and y in to the triangulation.  The
// returned index can be used to add edges involving this point to the
// constrained triangulation after all points have been added.
func (t *Triangulation) AddPoint(x, y float32) (index int) {
	pt := mgl32.Vec2{x, y}
	// find our encompassing triangle: (linear search cause honestly)
	parentTriIs := []int{}
	for i := 0; i < len(t.Triangles); i += 3 {
		if pointInTriangle(pt,
			t.Verts[t.Triangles[i]],
			t.Verts[t.Triangles[i+1]],
			t.Verts[t.Triangles[i+2]]) {
			parentTriIs = append(parentTriIs, i)
		}
	}
	if len(parentTriIs) <= 0 {
		// todo: handle this and other panics as a returned error
		panic("point out-of-bounds")
	}
	if len(parentTriIs) > 2 {
		// point is a duplicate
		for dupI := 0; dupI < len(t.Verts); dupI++ {
			if t.Verts[dupI].Sub(mgl32.Vec2{x, y}).Len() < 1e-6 {
				return dupI
			}
		}
		// this shouldn't be reached:
		return -1
	}
	ptI := len(t.Verts)
	t.Verts = append(t.Verts, pt)
	// indices into the dirty set of t.Triangles:
	checkTriangles := []int{}
	if len(parentTriIs) == 2 {
		// split 2 tris => 4 tris
		// find the clockwise quad points:
		quad := t.getSharedQuad(parentTriIs[0], parentTriIs[1])
		edge := [2]int{}
		// sorted common edge:
		if quad[1] > quad[3] {
			edge[0] = quad[3]
			edge[1] = quad[1]
		} else {
			edge[0] = quad[1]
			edge[1] = quad[3]
		}
		// find old edge
		oldEdgeI := -1
		for i := 0; i < len(t.Edges); i += 2 {
			if t.Edges[i] == edge[0] && t.Edges[i+1] == edge[1] {
				oldEdgeI = i
				break
			}
		}
		// generate new tris and edges
		t.Triangles = append(t.Triangles[:parentTriIs[0]],
			append([]int{quad[0], quad[1], ptI}, // 0
				t.Triangles[parentTriIs[0]+3:]...)...)
		t.Triangles = append(t.Triangles[:parentTriIs[1]],
			append([]int{quad[1], quad[2], ptI}, // 1
				t.Triangles[parentTriIs[1]+3:]...)...)
		t.Triangles = append(t.Triangles,
			quad[2], quad[3], ptI, // 2
			quad[3], quad[0], ptI) // 3
		t.Edges = append(t.Edges[:oldEdgeI],
			// our edge sortedness is guaranteed here because ptI is the
			// largest index
			append([]int{quad[1], ptI},
				t.Edges[oldEdgeI+2:]...)...)
		t.Edges = append(t.Edges,
			quad[2], ptI,
			quad[3], ptI,
			quad[0], ptI)
		t.fixed = append(t.fixed, false, false, false)
		checkTriangles = append(checkTriangles,
			parentTriIs[0], parentTriIs[1],
			len(t.Triangles)-6, len(t.Triangles)-3)
	} else {
		// split 1 tri => 3 tris
		triI := parentTriIs[0]
		triVs := []int{
			t.Triangles[triI], t.Triangles[triI+1], t.Triangles[triI+2]}
		t.Triangles = append(t.Triangles[:triI],
			append([]int{triVs[0], triVs[1], ptI},
				t.Triangles[triI+3:]...)...)
		t.Triangles = append(t.Triangles,
			triVs[1], triVs[2], ptI,
			triVs[2], triVs[0], ptI)
		t.Edges = append(t.Edges,
			triVs[1], ptI,
			triVs[2], ptI,
			triVs[0], ptI)
		t.fixed = append(t.fixed, false, false, false)
		checkTriangles = append(checkTriangles,
			triI, len(t.Triangles)-6, len(t.Triangles)-3)
	}
	for len(checkTriangles) > 0 {
		triI := checkTriangles[0]
		triV := []int{
			t.Triangles[triI], t.Triangles[triI+1], t.Triangles[triI+2]}
		// for all edges
		for i := 0; i < 3; i++ {
			edge := []int{triV[(i+1)%3], triV[i]}
			sortedEdge := []int{edge[0], edge[1]}
			if edge[0] > edge[1] {
				sortedEdge = []int{edge[1], edge[0]}
			}
			// find common edge index:
			edgeI := 0
			for ; edgeI < len(t.Edges); edgeI += 2 {
				if t.Edges[edgeI] == sortedEdge[0] &&
					t.Edges[edgeI+1] == sortedEdge[1] {
					break
				}
			}
			// if the edge is locked, give up now (at this step, this only
			// applies to the initial boundary edges)
			if t.fixed[edgeI/2] {
				continue
			}
			// find the triangle on the other side
			found := false
			otherTriI := -1
			for n := 0; n < len(t.Triangles); n += 3 {
				if n == triI {
					continue
				}
				if t.Triangles[n] == edge[0] && t.Triangles[n+1] == edge[1] {
					found = true
					otherTriI = n
					break
				}
				if t.Triangles[n+1] == edge[0] && t.Triangles[n+2] == edge[1] {
					found = true
					otherTriI = n
					break
				}
				if t.Triangles[n+2] == edge[0] && t.Triangles[n] == edge[1] {
					found = true
					otherTriI = n
					break
				}
			}
			// no neighbor?
			if !found {
				continue
			}
			// circumcircle test:
			quad := t.getSharedQuad(triI, otherTriI)
			a := t.Verts[quad[0]]
			b := t.Verts[quad[1]]
			c := t.Verts[quad[2]]
			d := t.Verts[quad[3]]
			sign := (mgl32.Mat4{
				d[0], d[1], d[0]*d[0] + d[1]*d[1], 1,
				c[0], c[1], c[0]*c[0] + c[1]*c[1], 1,
				b[0], b[1], b[0]*b[0] + b[1]*b[1], 1,
				a[0], a[1], a[0]*a[0] + a[1]*a[1], 1,
			}).Det()
			// if the determinant is too close to 0, we'll get stuck in a cycle
			if sign > 1e-7 {
				// flip: BD => AC
				t.Triangles = append(t.Triangles[:triI],
					append([]int{quad[0], quad[1], quad[2]},
						t.Triangles[triI+3:]...)...)
				t.Triangles = append(t.Triangles[:otherTriI],
					append([]int{quad[0], quad[2], quad[3]},
						t.Triangles[otherTriI+3:]...)...)
				newEdge := []int{quad[0], quad[2]}
				if newEdge[0] > newEdge[1] {
					newEdge = []int{quad[2], quad[0]}
				}
				t.Edges = append(t.Edges[:edgeI],
					append(newEdge,
						t.Edges[edgeI+2:]...)...)
				checkTriangles = append(checkTriangles, triI, otherTriI)
				break
			}
		}
		checkTriangles = checkTriangles[1:]
	}
	return ptI
}

// AddEdge forces an edge to exist in the triangulation.  This edge is then
// guaranteed to exist in the triangulation, unless a successive call to AddEdge
// specifies an edge that intersects this one.  If an intersecting edge is later
// specified, the later edge will "win".
func (t *Triangulation) AddEdge(indexA, indexB int) {
	if indexA == indexB {
		panic("bad graph")
	}
	edge := []int{indexA, indexB}
	if edge[0] > edge[1] {
		edge = []int{indexB, indexA}
	}
	edgeExists := false
	for i := 0; i < len(t.Edges); i += 2 {
		if t.Edges[i] == edge[0] && t.Edges[i+1] == edge[1] {
			edgeExists = true
			t.fixed[i/2] = true
			break
		}
	}
	if edgeExists {
		return
	}
	crossedTri := []int{}
	crossedTriI := -1
	for i := 0; i < len(t.Triangles); i += 3 {
		if t.Triangles[i] == edge[0] {
			if pointInAngle(t.Verts[edge[1]],
				t.Verts[t.Triangles[i]],
				t.Verts[t.Triangles[i+1]],
				t.Verts[t.Triangles[i+2]]) {
				crossedTri = []int{t.Triangles[i],
					t.Triangles[i+1],
					t.Triangles[i+2]}
				crossedTriI = i
				break
			}
		}
		if t.Triangles[i+1] == edge[0] {
			if pointInAngle(t.Verts[edge[1]],
				t.Verts[t.Triangles[i+1]],
				t.Verts[t.Triangles[i+2]],
				t.Verts[t.Triangles[i]]) {
				crossedTri = []int{t.Triangles[i+1],
					t.Triangles[i+2],
					t.Triangles[i]}
				crossedTriI = i
				break
			}
		}
		if t.Triangles[i+2] == edge[0] {
			if pointInAngle(t.Verts[edge[1]],
				t.Verts[t.Triangles[i+2]],
				t.Verts[t.Triangles[i]],
				t.Verts[t.Triangles[i+1]]) {
				crossedTri = []int{t.Triangles[i+2],
					t.Triangles[i],
					t.Triangles[i+1]}
				crossedTriI = i
				break
			}
		}
	}
	ptA := t.Verts[edge[0]]
	ptB := t.Verts[edge[1]]
	ptsU := []int{crossedTri[1]}
	ptsL := []int{crossedTri[2]}
	deadTriIs := []int{crossedTriI}
	deadEdges := []int{}
	t.newTris = []int{}
	t.newEdges = []int{}
	ringEdges := []int{crossedTri[0], crossedTri[1], crossedTri[0], crossedTri[2]}
	for {
		// get opposite triangle:
		otherTriI := -1
		otherVertI := -1
		for n := 0; n < len(t.Triangles); n += 3 {
			if n == crossedTriI {
				continue
			}
			if t.Triangles[n] == crossedTri[2] && t.Triangles[n+1] == crossedTri[1] {
				otherTriI = n
				otherVertI = t.Triangles[n+2]
				break
			}
			if t.Triangles[n+1] == crossedTri[2] && t.Triangles[n+2] == crossedTri[1] {
				otherTriI = n
				otherVertI = t.Triangles[n]
				break
			}
			if t.Triangles[n+2] == crossedTri[2] && t.Triangles[n] == crossedTri[1] {
				otherTriI = n
				otherVertI = t.Triangles[n+1]
				break
			}
		}
		deadTriIs = append(deadTriIs, otherTriI)
		deadEdges = append(deadEdges, crossedTri[1], crossedTri[2])
		if len(deadTriIs) > 1e5 {
			// in this case, we've either managed to loop around a small set of
			// triangles (bad graph), or the edge is actually crossing 10k tris
			panic("probable infinite loop detected")
		}
		if otherVertI == edge[1] {
			ringEdges = append(ringEdges, crossedTri[1], otherVertI, crossedTri[2], otherVertI)
			break
		}
		oppositePt := t.Verts[otherVertI]
		ptSide := (ptB[0]-ptA[0])*(oppositePt[1]-ptA[1]) -
			(ptB[1]-ptA[1])*(oppositePt[0]-ptA[0])
		if ptSide > 1e-12 { // above
			ptsU = append(ptsU, otherVertI)
			ringEdges = append(ringEdges, crossedTri[1], otherVertI)
			crossedTri = []int{crossedTri[1], otherVertI, crossedTri[2]}
			crossedTriI = otherTriI
		} else if ptSide < -1e-12 { // below
			ptsL = append(ptsL, otherVertI)
			ringEdges = append(ringEdges, crossedTri[2], otherVertI)
			crossedTri = []int{crossedTri[2], crossedTri[1], otherVertI}
			crossedTriI = otherTriI
		} else { // incident
			t.AddEdge(otherVertI, edge[1])
			edge[1] = otherVertI
			break
		}
	}
	t.retriangulate(ringEdges, ptsU, edge)
	edge = []int{edge[1], edge[0]}
	t.retriangulate(ringEdges, ptsL, edge)
	if edge[0] > edge[1] {
		edge = []int{edge[1], edge[0]}
	}
	t.newEdges = t.newEdges[:len(t.newEdges)-2]
	for i := 0; i < len(deadTriIs); i++ {
		t.Triangles[deadTriIs[i]] = t.newTris[3*i]
		t.Triangles[deadTriIs[i]+1] = t.newTris[3*i+1]
		t.Triangles[deadTriIs[i]+2] = t.newTris[3*i+2]
	}
	for i := 0; i < len(deadEdges); i += 2 {
		deadEdge := []int{deadEdges[i], deadEdges[i+1]}
		if deadEdges[i] > deadEdges[i+1] {
			deadEdge = []int{deadEdges[i+1], deadEdges[i]}
		}
		newEdge := []int{t.newEdges[i], t.newEdges[i+1]}
		if t.newEdges[i] > t.newEdges[i+1] {
			newEdge = []int{t.newEdges[i+1], t.newEdges[i]}
		}
		last := i == len(deadEdges)-2
		for j := 0; j < len(t.Edges); j += 2 {
			if deadEdge[0] == t.Edges[j] && deadEdge[1] == t.Edges[j+1] {
				t.Edges[j] = newEdge[0]
				t.Edges[j+1] = newEdge[1]
				if last {
					t.fixed[j/2] = true
				}
				break
			}
		}
	}
}

func (t *Triangulation) retriangulate(ringEdges, vertIs, edgeIs []int) {
	cI := -1
	if len(vertIs) > 1 {
		cI = vertIs[0]
		c := t.Verts[cI]
		// maintaining sanity about geometric orientation here is
		// a bit tricky
		a := t.Verts[edgeIs[0]]
		b := t.Verts[edgeIs[1]]
		// find the closest vert to the edge:
		for i := 1; i < len(vertIs); i++ {
			d := t.Verts[vertIs[i]]
			sign := (mgl32.Mat4{
				a[0], a[1], a[0]*a[0] + a[1]*a[1], 1,
				b[0], b[1], b[0]*b[0] + b[1]*b[1], 1,
				c[0], c[1], c[0]*c[0] + c[1]*c[1], 1,
				d[0], d[1], d[0]*d[0] + d[1]*d[1], 1,
			}).Det()
			if sign > 0 {
				cI = vertIs[i]
				c = t.Verts[cI]
			}
		}
		// partition vertIs into left/right of c:
		// left/right is determined by following the edge ring defined
		// by vertIs split on c
		leftVertIs := []int{}
		rightVertIs := []int{}
		{
			ringI := edgeIs[0]
			prevRingI := -1
			right := false
			ptInRing := func(idx int) (onRing bool, onEdge bool) {
				for _, vI := range vertIs {
					if vI == idx {
						return true, false
					}
				}
				if idx == edgeIs[1] || idx == edgeIs[0] {
					return true, true
				}
				return false, false
			}
			for {
				inRing := false
				shouldBreak := false
				testI := -1
				for i := 0; i < len(ringEdges); i += 2 {
					if ringEdges[i] == ringI {
						testI = ringEdges[i+1]
					}
					if ringEdges[i+1] == ringI {
						testI = ringEdges[i]
					}
					if testI >= 0 && prevRingI != testI {
						inRing, shouldBreak = ptInRing(testI)
						if shouldBreak {
							break
						}
						if inRing {
							prevRingI = ringI
							ringI = testI
							if ringI == cI {
								right = true
							} else if right {
								rightVertIs = append(rightVertIs, ringI)
							} else {
								leftVertIs = append(leftVertIs, ringI)
							}
							break
						}
					}
				}
				if shouldBreak {
					break
				}
				if ringI == prevRingI {
					panic("finding edge in ring won't terminate")
				}
			}
		}
		t.retriangulate(ringEdges, leftVertIs, []int{edgeIs[0], cI})
		t.retriangulate(ringEdges, rightVertIs, []int{cI, edgeIs[1]})
	}
	if len(vertIs) > 0 {
		if cI == -1 {
			cI = vertIs[0]
		}
		t.newTris = append(t.newTris, edgeIs[1], edgeIs[0], cI)
		t.newEdges = append(t.newEdges, edgeIs[0], edgeIs[1])
	}
}

// getSharedQuad returns the quad a,b,c,d defined by triangles a,b,c and c,b,d;
// the returned quad array has the verts of the shared edge at quad[1] and
// quad[3].
func (t *Triangulation) getSharedQuad(triA, triB int) [4]int {
	quad := [4]int{}
	tris := [2]int{triA, triB}
	for n := 0; n < 2; n++ {
		triA = tris[n]
		triB = tris[1-n]
		for j := 0; j < 3; j++ {
			m := t.Triangles[triA+j]
			found := false
			for i := 0; i < 3; i++ {
				if t.Triangles[triB+i] == m {
					found = true
					break
				}
			}
			if !found {
				quad[n*2] = m
				quad[n*2+1] = t.Triangles[triA+((j+1)%3)]
				break
			}
		}
	}
	return quad
}

// getBarycentric returns the two barycentric components of the triangle abc for
// p relative to b and c
func getBarycentric(p, a, b, c mgl32.Vec2) (float32, float32) {
	v0 := c.Sub(a)
	v1 := b.Sub(a)
	v2 := p.Sub(a)
	dot00 := v0.Dot(v0)
	dot01 := v0.Dot(v1)
	dot02 := v0.Dot(v2)
	dot11 := v1.Dot(v1)
	dot12 := v1.Dot(v2)
	norm := 1 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * norm
	v := (dot00*dot12 - dot01*dot02) * norm
	return u, v
}

// pointInTriangle returns true if p is inside the triangle abc
func pointInTriangle(p, a, b, c mgl32.Vec2) bool {
	u, v := getBarycentric(p, a, b, c)
	// all those dot products really accumulate the rounding error!
	return u >= -1e-6 && v >= -1e-6 && u+v-1 < 1e-6
}

// pointInAngle returns true if p is inside the angle defined by lines ab and ac
func pointInAngle(p, a, b, c mgl32.Vec2) bool {
	u, v := getBarycentric(p, a, b, c)
	return u >= -1e-6 && v >= -1e-6
}
