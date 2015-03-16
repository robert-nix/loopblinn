// portions copied from https://github.com/go-gl/examples
package main

import (
	"errors"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"io/ioutil"
	"log"
	"math"
	"os"
	"runtime"
	"time"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
	"github.com/go-gl/mathgl/mgl32"

	"code.google.com/p/freetype-go/freetype/raster"
	"code.google.com/p/freetype-go/freetype/truetype"
)

func init() {
	// GLFW event handling must run on the main OS thread
	runtime.LockOSThread()

	frameTimeIndex = 0
}

var frameTimes [512]time.Time
var frameTimeIndex int
var frameBeginTime time.Time
var zeroTime time.Time
var frameTime float64

func frameRate() float32 {
	var sumSeconds float32
	var nonZero int
	prevTime := frameTimes[frameTimeIndex&511]
	for i := 1; i < 512; i++ {
		currTime := frameTimes[(frameTimeIndex+i)&511]
		if !prevTime.IsZero() && !currTime.IsZero() {
			nonZero++
			sumSeconds += float32(currTime.Sub(prevTime).Seconds())
		}
		prevTime = currTime
	}
	if nonZero == 0 {
		return 0
	}
	return float32(1.0) / (sumSeconds / float32(nonZero))
}

func beginFrame() {
	frameBeginTime = time.Now()
}

func endFrame() {
	frameEndTime := time.Now()
	frameTime = float64(frameEndTime.Sub(frameBeginTime).Seconds()) * 1000.0
	frameTimes[frameTimeIndex&511] = frameEndTime
	frameTimeIndex++
}

var font *truetype.Font
var glyph *truetype.GlyphBuf

type GlyphMesh struct {
	// x, y; u, v; ib
	positions []float32
	uvs       []int8
	indices   []int16
}

var glyphMesh GlyphMesh

func loadFont(path string) {
	file, err := os.Open(path)
	if err != nil {
		panic(err)
	}

	buf, err := ioutil.ReadAll(file)
	if err != nil {
		panic(err)
	}

	font, err = truetype.Parse(buf)
	if err != nil {
		panic(err)
	}

	glyph = truetype.NewGlyphBuf()
}

const (
	uvBeginConvex = iota
	uvMidConvex
	uvEndConvex
	uvBeginConcave
	uvMidConcave
	uvEndConcave
	uvExterior
	uvInterior
)

func loadGlyph(r rune) {
	err := glyph.Load(font, 65536, font.Index(r), truetype.NoHinting)
	if err != nil {
		panic(err)
	}
	fmt.Println(glyph.Point)

	// preprocessing
	type point struct {
		x, y float32
		on   bool
	}
	type glyphInfo struct {
		loops [][]point
	}
	g := glyphInfo{}
	i := 0
	fmt.Println(glyph.End)
	for _, end := range glyph.End {
		loop := []point{}
		loopI := 0
		loopStartI := 0
		foundStartPoint := false
		for i < end {
			p := glyph.Point[i]
			// (0, 65536) => (0.0, 1.0)
			x := float32(p.X) / 65536
			y := float32(p.Y) / 65536
			on := 0 != (p.Flags & 1)
			fmt.Println("pt", i, x, y, on)
			if on && !foundStartPoint {
				loopStartI = loopI
			}
			loop = append(loop, point{x, y, on})
			loopI++
			i++
		}
		loop = append(loop[loopStartI:], loop[:loopStartI]...)
		fmt.Println("new loop")
		if len(loop) > 1 {
			g.loops = append(g.loops, loop)
		}
	}

	// Triangulation!
	i = 0
	xMin := float32(glyph.B.XMin) / 65536
	xMax := float32(glyph.B.XMax) / 65536
	yMin := float32(glyph.B.YMin) / 65536
	yMax := float32(glyph.B.YMax) / 65536
	width := xMax - xMin
	height := yMax - yMin
	// 10% expansion:
	xMin -= width * 0.05
	xMax += width * 0.05
	yMin -= height * 0.05
	yMax += height * 0.05
	fmt.Println("bounds", xMin, xMax, yMin, yMax)
	// define points and bezier triangles:
	glyphMesh = GlyphMesh{}
	positions := make([]float32, 0)
	uvs := make([]int8, 0)
	indices := make([]int16, 0)
	lines := []int16{}
	addVert := func(x, y float32, uv int8) {
		positions = append(positions, x, y)
		uvs = append(uvs, uv)
		fmt.Println("v", x, y, uv)
	}
	n := int16(0)
	addIndex := func(idx int16) {
		indices = append(indices, idx)
		fmt.Println("i", idx)
		n++
	}
	addLine := func(a, b int16) {
		lines = append(lines, a, b)
	}
	fmt.Println(g)
	for _, loop := range g.loops {
		first := true
		firstI := int16(-1)
		firstX := float32(0)
		firstY := float32(0)
		prevX := float32(0)
		prevY := float32(0)
		prevOn := true
		var x, y float32
		for _, pt := range loop {
			x := pt.x
			y := pt.y
			on := pt.on
			fmt.Println("pt", i, x, y, on)
			if on {
				// on => 0,0 / 1,1 uv
				if !first {
					if !prevOn {
						// close last bezier:
						addVert(x, y, uvEndConvex)
						addIndex(n)
					} else {
						n++
						addLine(n-1, n)
					}
				}
				// gen 0,0 uv
				if firstI < 0 {
					firstI = n
				}
				addVert(x, y, uvBeginConvex)
			} else {
				if first {
					panic("NYI: rewind the loop to start with an 'on' point")
				}
				if !prevOn {
					// gen midpoint
					midX := (x + prevX) / 2
					midY := (y + prevY) / 2
					// close last:
					addVert(midX, midY, uvEndConvex)
					addIndex(n)
					// open next:
					addVert(midX, midY, uvBeginConvex)
					addIndex(n)
				} else {
					// open current:
					addIndex(n)
				}
				addVert(x, y, uvMidConvex)
				addIndex(n)
			}
			if first {
				firstX = x
				firstY = y
				first = false
			}
			prevX = x
			prevY = y
			prevOn = on
		}
		if !prevOn {
			fmt.Println("closing loop?")
			addVert(firstX, firstY, uvEndConvex)
			addIndex(n)
		} else {
			if x != firstX || y != firstY {
				addLine(n, firstI)
			}
			n++
		}
	}
	fmt.Println("nVertices:", len(positions)/2)
	fmt.Println("nIndices:", len(indices))

	// now we must create a proper tessellation of the graph, using constrained
	// Delauney triangulation
	//
	// for the graph's initial state, we use the 10% enlarged bounds from
	// earlier:
	dtVerts := []mgl32.Vec2{
		{xMin, yMin},
		{xMin, yMax},
		{xMax, yMin},
		{xMax, yMax},
	}
	dtEdges := []int{
		// by convention, the first index is less than the second
		0, 1,
		0, 2,
		1, 2,
		1, 3,
		2, 3,
	}
	dtTriangles := []int{
		0, 1, 2,
		2, 1, 3,
	}
	dtFixed := []bool{
		// the border edges are fixed:
		true, true, false, true, true,
	}
	fmt.Println(dtFixed, dtEdges)
	getSharedQuad := func(triA, triB int) [4]int {
		quad := [4]int{}
		tris := [2]int{triA, triB}
		for n := 0; n < 2; n++ {
			triA = tris[n]
			triB = tris[1-n]
			for j := 0; j < 3; j++ {
				m := dtTriangles[triA+j]
				found := false
				for i := 0; i < 3; i++ {
					if dtTriangles[triB+i] == m {
						found = true
						break
					}
				}
				if !found {
					quad[n*2] = m
					quad[n*2+1] = dtTriangles[triA+((j+1)%3)]
					break
				}
			}
		}
		return quad
	}
	traceIdx := 0
	trace := func(triOfInterest int, fatal bool) {
		if !fatal {
			return
		}
		r := raster.NewRasterizer(1024, 1024)
		r.UseNonZeroWinding = true
		img := image.NewRGBA(image.Rect(0, 0, 1024, 1024))
		p := raster.NewRGBAPainter(img)
		for n := 0; n < len(dtTriangles); n += 3 {
			c := colorList[(n/3)%len(colorList)]
			p.SetColor(color.RGBA{uint8(c[0]), uint8(c[1]), uint8(c[2]), 255})
			p0 := dtVerts[dtTriangles[n+0]]
			p1 := dtVerts[dtTriangles[n+1]]
			p2 := dtVerts[dtTriangles[n+2]]
			pa := raster.Path{}
			pa.Start(raster.Point{
				raster.Fix32(int((0.1 + p0[0]) * 256 * 1200)),
				raster.Fix32(256000 - int((0.3+p0[1])*256*1200))})
			pa.Add1(raster.Point{
				raster.Fix32(int((0.1 + p1[0]) * 256 * 1200)),
				raster.Fix32(256000 - int((0.3+p1[1])*256*1200))})
			pa.Add1(raster.Point{
				raster.Fix32(int((0.1 + p2[0]) * 256 * 1200)),
				raster.Fix32(256000 - int((0.3+p2[1])*256*1200))})
			pa.Add1(raster.Point{
				raster.Fix32(int((0.1 + p0[0]) * 256 * 1200)),
				raster.Fix32(256000 - int((0.3+p0[1])*256*1200))})
			nw := raster.Fix32(256)
			if n == triOfInterest {
				nw = 1024
			}
			r.AddStroke(pa, nw, nil, nil)
			r.Rasterize(p)
			r.Clear()
			mp := p0.Add(p1).Add(p2).Mul(1.0 / 3.0)
			mpX := raster.Fix32(int((0.1 + mp[0]) * 256 * 1200))
			mpY := raster.Fix32(256000 - int((0.3+mp[1])*256*1200))
			r.Start(raster.Point{mpX, mpY + 256})
			r.Add2(
				raster.Point{mpX + 256, mpY + 256},
				raster.Point{mpX + 256, mpY})
			r.Add2(
				raster.Point{mpX + 256, mpY - 256},
				raster.Point{mpX, mpY - 256})
			r.Add2(
				raster.Point{mpX - 256, mpY - 256},
				raster.Point{mpX - 256, mpY})
			r.Add2(
				raster.Point{mpX - 256, mpY + 256},
				raster.Point{mpX, mpY + 256})
			r.Rasterize(p)
			r.Clear()
		}
		outFile, err := os.Create(fmt.Sprintf("trace_%03d.png", traceIdx))
		traceIdx++
		if err != nil {
			panic(err)
		}
		png.Encode(outFile, img)
		outFile.Close()
	}
	addPoint := func(x, y float32) int {
		fmt.Println("addPoint", x, y)
		pt := mgl32.Vec2{x, y}
		// find our encompassing triangle: (linear search cause honestly)
		parentTriIs := []int{}
		for i := 0; i < len(dtTriangles); i += 3 {
			if pointInTriangle(pt,
				dtVerts[dtTriangles[i]],
				dtVerts[dtTriangles[i+1]],
				dtVerts[dtTriangles[i+2]]) {
				parentTriIs = append(parentTriIs, i)
			}
		}
		if len(parentTriIs) <= 0 {
			panic("point out-of-bounds")
		}
		if len(parentTriIs) > 2 {
			// point is a duplicate
			fmt.Println("duplicate pt", pt)
			for dupI := 0; dupI < len(dtVerts); dupI++ {
				if dtVerts[dupI].Sub(mgl32.Vec2{x, y}).Len() < 1e-6 {
					return dupI
				}
			}
			return -1
		}
		ptI := len(dtVerts)
		dtVerts = append(dtVerts, pt)
		// indices into the dirty set of dtTriangles:
		checkTriangles := []int{}
		if len(parentTriIs) == 2 {
			// split 2 tris => 4 tris
			// find the clockwise quad points:
			quad := getSharedQuad(parentTriIs[0], parentTriIs[1])
			fmt.Println("parent quad:", quad)
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
			for i := 0; i < len(dtEdges); i += 2 {
				if dtEdges[i] == edge[0] && dtEdges[i+1] == edge[1] {
					oldEdgeI = i
					break
				}
			}
			// generate new tris and edges
			dtTriangles = append(dtTriangles[:parentTriIs[0]],
				append([]int{quad[0], quad[1], ptI}, // 0
					dtTriangles[parentTriIs[0]+3:]...)...)
			dtTriangles = append(dtTriangles[:parentTriIs[1]],
				append([]int{quad[1], quad[2], ptI}, // 1
					dtTriangles[parentTriIs[1]+3:]...)...)
			dtTriangles = append(dtTriangles,
				quad[2], quad[3], ptI, // 2
				quad[3], quad[0], ptI) // 3
			dtEdges = append(dtEdges[:oldEdgeI],
				// our edge sortedness is guaranteed here because ptI is the
				// largest index
				append([]int{quad[1], ptI},
					dtEdges[oldEdgeI+2:]...)...)
			dtEdges = append(dtEdges,
				quad[2], ptI,
				quad[3], ptI,
				quad[0], ptI)
			dtFixed = append(dtFixed, false, false, false)
			checkTriangles = append(checkTriangles,
				parentTriIs[0], parentTriIs[1],
				len(dtTriangles)-6, len(dtTriangles)-3)
		} else {
			// split 1 tri => 3 tris
			triI := parentTriIs[0]
			triVs := []int{
				dtTriangles[triI], dtTriangles[triI+1], dtTriangles[triI+2]}
			dtTriangles = append(dtTriangles[:triI],
				append([]int{triVs[0], triVs[1], ptI},
					dtTriangles[triI+3:]...)...)
			dtTriangles = append(dtTriangles,
				triVs[1], triVs[2], ptI,
				triVs[2], triVs[0], ptI)
			dtEdges = append(dtEdges,
				triVs[1], ptI,
				triVs[2], ptI,
				triVs[0], ptI)
			dtFixed = append(dtFixed, false, false, false)
			checkTriangles = append(checkTriangles,
				triI, len(dtTriangles)-6, len(dtTriangles)-3)
		}
		// fmt.Println(parentTriIs)
		// fmt.Println(dtVerts)
		// fmt.Println(dtTriangles)
		for len(checkTriangles) > 0 {
			triI := checkTriangles[0]
			trace(triI, false)
			triV := []int{
				dtTriangles[triI], dtTriangles[triI+1], dtTriangles[triI+2]}
			// for all edges
			for i := 0; i < 3; i++ {
				edge := []int{triV[(i+1)%3], triV[i]}
				sortedEdge := []int{edge[0], edge[1]}
				if edge[0] > edge[1] {
					sortedEdge = []int{edge[1], edge[0]}
				}
				// find common edge index:
				edgeI := 0
				for ; edgeI < len(dtEdges); edgeI += 2 {
					if dtEdges[edgeI] == sortedEdge[0] &&
						dtEdges[edgeI+1] == sortedEdge[1] {
						break
					}
				}
				// if the edge is locked, give up now
				if dtFixed[edgeI/2] {
					continue
				}
				// find the triangle on the other side
				found := false
				otherTriI := -1
				for n := 0; n < len(dtTriangles); n += 3 {
					if n == triI {
						continue
					}
					if dtTriangles[n] == edge[0] && dtTriangles[n+1] == edge[1] {
						found = true
						otherTriI = n
						break
					}
					if dtTriangles[n+1] == edge[0] && dtTriangles[n+2] == edge[1] {
						found = true
						otherTriI = n
						break
					}
					if dtTriangles[n+2] == edge[0] && dtTriangles[n] == edge[1] {
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
				quad := getSharedQuad(triI, otherTriI)
				a := dtVerts[quad[0]]
				b := dtVerts[quad[1]]
				c := dtVerts[quad[2]]
				d := dtVerts[quad[3]]
				sign := (mgl32.Mat4{
					d[0], d[1], d[0]*d[0] + d[1]*d[1], 1,
					c[0], c[1], c[0]*c[0] + c[1]*c[1], 1,
					b[0], b[1], b[0]*b[0] + b[1]*b[1], 1,
					a[0], a[1], a[0]*a[0] + a[1]*a[1], 1,
				}).Det()
				if mgl32.Abs(sign) < 1e-7 {
					// don't bother!
					continue
				}
				if sign > 0 {
					// flip: BD => AC
					fmt.Println("flipping quad", quad, "det", sign)
					dtTriangles = append(dtTriangles[:triI],
						append([]int{quad[0], quad[1], quad[2]},
							dtTriangles[triI+3:]...)...)
					dtTriangles = append(dtTriangles[:otherTriI],
						append([]int{quad[0], quad[2], quad[3]},
							dtTriangles[otherTriI+3:]...)...)
					newEdge := []int{quad[0], quad[2]}
					if newEdge[0] > newEdge[1] {
						newEdge = []int{quad[2], quad[0]}
					}
					dtEdges = append(dtEdges[:edgeI],
						append(newEdge,
							dtEdges[edgeI+2:]...)...)
					checkTriangles = append(checkTriangles, triI, otherTriI)
					break
				}
			}
			checkTriangles = checkTriangles[1:]
		}
		return ptI
	}
	var forceEdge func(vertAI, vertBI int)
	forceEdge = func(vertAI, vertBI int) {
		fmt.Println("edges:", dtEdges)
		fmt.Println("forcing", dtVerts[vertAI], dtVerts[vertBI])
		if vertAI == vertBI {
			panic("bad graph yo")
		}
		edge := []int{vertAI, vertBI}
		if edge[0] > edge[1] {
			edge = []int{vertBI, vertAI}
		}
		edgeExists := false
		for i := 0; i < len(dtEdges); i += 2 {
			if dtEdges[i] == edge[0] && dtEdges[i+1] == edge[1] {
				edgeExists = true
				dtFixed[i/2] = true
				break
			}
		}
		if edgeExists {
			fmt.Println("duplicate edge")
			return
		}
		crossedTri := []int{}
		crossedTriI := -1
		for i := 0; i < len(dtTriangles); i += 3 {
			if dtTriangles[i] == edge[0] {
				if pointInAngle(dtVerts[edge[1]],
					dtVerts[dtTriangles[i]],
					dtVerts[dtTriangles[i+1]],
					dtVerts[dtTriangles[i+2]]) {
					crossedTri = []int{dtTriangles[i],
						dtTriangles[i+1],
						dtTriangles[i+2]}
					crossedTriI = i
					break
				}
			}
			if dtTriangles[i+1] == edge[0] {
				if pointInAngle(dtVerts[edge[1]],
					dtVerts[dtTriangles[i+1]],
					dtVerts[dtTriangles[i+2]],
					dtVerts[dtTriangles[i]]) {
					crossedTri = []int{dtTriangles[i+1],
						dtTriangles[i+2],
						dtTriangles[i]}
					crossedTriI = i
					break
				}
			}
			if dtTriangles[i+2] == edge[0] {
				if pointInAngle(dtVerts[edge[1]],
					dtVerts[dtTriangles[i+2]],
					dtVerts[dtTriangles[i]],
					dtVerts[dtTriangles[i+1]]) {
					crossedTri = []int{dtTriangles[i+2],
						dtTriangles[i],
						dtTriangles[i+1]}
					crossedTriI = i
					break
				}
			}
		}
		ptA := dtVerts[edge[0]]
		ptB := dtVerts[edge[1]]
		fmt.Println("edge points:", ptA, ptB)
		fmt.Println("edge is:", edge)
		ptsU := []int{crossedTri[1]}
		ptsL := []int{crossedTri[2]}
		deadTriIs := []int{crossedTriI}
		newTris := []int{}
		deadEdges := []int{}
		newEdges := []int{}
		ringEdges := []int{crossedTri[0], crossedTri[1], crossedTri[0], crossedTri[2]}
		fmt.Println("target", dtVerts[edge[1]])
		for {
			// get opposite triangle:
			otherTriI := -1
			otherVertI := -1
			fmt.Println(crossedTri)
			trace(crossedTriI, false)
			fmt.Println("crossed triangle",
				dtVerts[crossedTri[0]], dtVerts[crossedTri[1]], dtVerts[crossedTri[2]])
			for n := 0; n < len(dtTriangles); n += 3 {
				if n == crossedTriI {
					continue
				}
				if dtTriangles[n] == crossedTri[2] && dtTriangles[n+1] == crossedTri[1] {
					otherTriI = n
					otherVertI = dtTriangles[n+2]
					break
				}
				if dtTriangles[n+1] == crossedTri[2] && dtTriangles[n+2] == crossedTri[1] {
					otherTriI = n
					otherVertI = dtTriangles[n]
					break
				}
				if dtTriangles[n+2] == crossedTri[2] && dtTriangles[n] == crossedTri[1] {
					otherTriI = n
					otherVertI = dtTriangles[n+1]
					break
				}
			}
			fmt.Println(otherTriI)
			fmt.Println(otherVertI)
			deadTriIs = append(deadTriIs, otherTriI)
			deadEdges = append(deadEdges, crossedTri[1], crossedTri[2])
			if len(deadTriIs) > 30 {
				panic("asd")
			}
			if otherVertI == edge[1] {
				ringEdges = append(ringEdges, crossedTri[1], otherVertI, crossedTri[2], otherVertI)
				break
			}
			if otherVertI == -1 {
				trace(crossedTriI, true)
			}
			oppositePt := dtVerts[otherVertI]
			ptSide := (ptB[0]-ptA[0])*(oppositePt[1]-ptA[1]) -
				(ptB[1]-ptA[1])*(oppositePt[0]-ptA[0])
			fmt.Println(ptSide)
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
				fmt.Println("recursing for incident edge")
				forceEdge(otherVertI, edge[1])
				edge[1] = otherVertI
				break
			}
		}
		var retriangulate func(vertIs, edgeIs []int)
		retriangulate = func(vertIs, edgeIs []int) {
			cI := -1
			if len(vertIs) > 1 {
				cI = vertIs[0]
				c := dtVerts[cI]
				// maintaining sanity about geometric orientation here is
				// a bit tricky
				a := dtVerts[edgeIs[0]]
				b := dtVerts[edgeIs[1]]
				// find the closest vert to the edge:
				for i := 1; i < len(vertIs); i++ {
					d := dtVerts[vertIs[i]]
					sign := (mgl32.Mat4{
						a[0], a[1], a[0]*a[0] + a[1]*a[1], 1,
						b[0], b[1], b[0]*b[0] + b[1]*b[1], 1,
						c[0], c[1], c[0]*c[0] + c[1]*c[1], 1,
						d[0], d[1], d[0]*d[0] + d[1]*d[1], 1,
					}).Det()
					fmt.Println(sign)
					if sign > 0 {
						cI = vertIs[i]
						c = dtVerts[cI]
					}
				}
				// partition vertIs into left/right of c:
				// left/right is determined by following the edge ring defined
				// by vertIs split on c
				leftVertIs := []int{}
				rightVertIs := []int{}
				{
					fmt.Println("ring edges", ringEdges)
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
						fmt.Println("ringI", ringI, "prevRingI", prevRingI, "testI", testI, "cI", cI, "right", right, "edge0", edgeIs[0], "edge1", edgeIs[1], "vertIs", vertIs, "leftVertIs", leftVertIs, "rightVertIs", rightVertIs)
						if ringI == prevRingI {
							panic("asd")
						}
					}
				}
				retriangulate(leftVertIs, []int{edgeIs[0], cI})
				retriangulate(rightVertIs, []int{cI, edgeIs[1]})
			}
			if len(vertIs) > 0 {
				if cI == -1 {
					cI = vertIs[0]
				}
				det := mgl32.Mat3FromRows(
					dtVerts[edgeIs[1]].Vec3(1),
					dtVerts[edgeIs[0]].Vec3(1),
					dtVerts[cI].Vec3(1)).Det()
				fmt.Println("adding tri", edgeIs[1], edgeIs[0], cI, "det:", det)
				newTris = append(newTris, edgeIs[1], edgeIs[0], cI)
				newEdges = append(newEdges, edgeIs[0], edgeIs[1])
			}
		}
		fmt.Println("ptsU:", ptsU)
		fmt.Println("ptsL:", ptsL)
		retriangulate(ptsU, edge)
		edge = []int{edge[1], edge[0]}
		retriangulate(ptsL, edge)
		if edge[0] > edge[1] {
			edge = []int{edge[1], edge[0]}
		}
		newEdges = newEdges[:len(newEdges)-2]
		fmt.Println("deadTriIs:", deadTriIs)
		fmt.Println("newTris:", newTris)
		fmt.Println("deadEdges:", deadEdges)
		fmt.Println("newEdges:", newEdges)
		for i := 0; i < len(deadTriIs); i++ {
			fmt.Println("deadTri:",
				dtTriangles[deadTriIs[i]],
				dtTriangles[deadTriIs[i]+1],
				dtTriangles[deadTriIs[i]+2])
			trace(deadTriIs[i], false)
			dtTriangles[deadTriIs[i]] = newTris[3*i]
			dtTriangles[deadTriIs[i]+1] = newTris[3*i+1]
			dtTriangles[deadTriIs[i]+2] = newTris[3*i+2]
			trace(deadTriIs[i], false)
		}
		for i := 0; i < len(deadEdges); i += 2 {
			deadEdge := []int{deadEdges[i], deadEdges[i+1]}
			if deadEdges[i] > deadEdges[i+1] {
				deadEdge = []int{deadEdges[i+1], deadEdges[i]}
			}
			newEdge := []int{newEdges[i], newEdges[i+1]}
			if newEdges[i] > newEdges[i+1] {
				newEdge = []int{newEdges[i+1], newEdges[i]}
			}
			last := i == len(deadEdges)-2
			for j := 0; j < len(dtEdges); j += 2 {
				if deadEdge[0] == dtEdges[j] && deadEdge[1] == dtEdges[j+1] {
					dtEdges[j] = newEdge[0]
					dtEdges[j+1] = newEdge[1]
					if last {
						dtFixed[j/2] = true
					}
					break
				}
			}
		}
	}
	srcToDtIs := []int{}
	for i := 0; i < len(positions); i += 2 {
		dtI := addPoint(positions[i+0], positions[i+1])
		srcToDtIs = append(srcToDtIs, dtI)
	}
	fmt.Println("making edges", indices, srcToDtIs)
	fmt.Println("lines", lines)
	for i := 0; i < len(indices); i += 3 {
		fmt.Println("force tri:", indices[i:i+3])
		forceEdge(srcToDtIs[int(indices[i+0])], srcToDtIs[int(indices[i+1])])
		forceEdge(srcToDtIs[int(indices[i+1])], srcToDtIs[int(indices[i+2])])
		forceEdge(srcToDtIs[int(indices[i+2])], srcToDtIs[int(indices[i+0])])
	}
	for i := 0; i < len(lines); i += 2 {
		forceEdge(srcToDtIs[int(lines[i])], srcToDtIs[int(lines[i+1])])
	}
	fmt.Println("lines", lines)
	testX := float32(0.40400696)
	testY := float32(0.65600586)
	pointInGlyph := func(q mgl32.Vec2) bool {
		xs := []float32{}
		interiors := []bool{}
		intersectQuad := func(p0, p1, p2 mgl32.Vec2) {
			a := p0[1]
			b := p1[1]
			c := p2[1]
			y := q[1]
			t0 := float32(0)
			t1 := float32(0)
			if !mgl32.FloatEqual(a-2*b+c, 0) {
				det := float32(math.Sqrt(float64(-a*c + a*y + b*b - 2*b*y + c*y)))
				denom := -a + 2*b - c
				t0 = (-det - a + b) / denom
				t1 = (det - a + b) / denom
			} else if mgl32.FloatEqual(a, 2*b-c) && !mgl32.FloatEqual(b-c, 0) {
				// tangent
				t0 = (2*b - c - y) / (2 * (b - c))
				t1 = -1 // substitute for -Inf
			} else {
				// straight colinear line
				return
			}
			if q[0] == testX && q[1] == testY {
				fmt.Println("q:", q, "p0,1,2:", p0, p1, p2, "ts:", t0, t1)
			}
			if t0 >= 0 && t0 <= 1 {
				// get x for t:
				x := (1-t0)*((1-t0)*p0[0]+t0*p1[0]) + t0*((1-t0)*p1[0]+t0*p2[0])
				if x < q[0] {
					fmt.Println("left x goto:", x)
					goto lt1
				}
				fmt.Println("quad t0:", t0, "x:", x)
				xs = append(xs, x)
				// to determine interiority, we need the vector derivative of
				// the bezier at t:
				dB := p1.Sub(p0).Mul(2 * (1 - t0)).Add(p2.Sub(p1).Mul(2 * t0))
				// also need det([[p2 - p0] [p1 - p0]])
				if dB[1] > 0 {
					fmt.Println("q is exterior")
					interiors = append(interiors, false)
				} else {
					fmt.Println("q is interior")
					interiors = append(interiors, true)
				}
			}
		lt1:
			if t1 >= 0 && t1 <= 1 {
				x := (1-t1)*((1-t1)*p0[0]+t1*p1[0]) + t1*((1-t1)*p1[0]+t1*p2[0])
				if x < q[0] {
					fmt.Println("left x:", x)
					return
				}
				fmt.Println("quad t1:", t1, "x:", x)
				xs = append(xs, x)
				dB := p1.Sub(p0).Mul(2 * (1 - t1)).Add(p2.Sub(p1).Mul(2 * t1))
				if dB[1] > 0 {
					fmt.Println("q is exterior")
					interiors = append(interiors, false)
				} else {
					fmt.Println("q is interior")
					interiors = append(interiors, true)
				}
			}
		}
		intersectLinear := func(p0, p1 mgl32.Vec2) {
			if p0[1] == p1[1] {
				// another segment will determine this
				return
			}
			t := (p0[1] - q[1]) / (p0[1] - p1[1])
			if t >= 0 && t <= 1 {
				x := (1-t)*p0[0] + t*p1[0]
				if x < q[0] {
					return
				}
				fmt.Println("linear t:", t, "x:", x)
				xs = append(xs, x)
				if p0[1] < p1[1] {
					fmt.Println("q is exterior")
					interiors = append(interiors, false)
				} else {
					fmt.Println("q is interior")
					interiors = append(interiors, true)
				}
			}
		}
		// can totally make this better with spatial sorting but that requires
		// writing code
		for _, loop := range g.loops {
			first := true
			firstX := float32(0)
			firstY := float32(0)
			prevX := float32(0)
			prevY := float32(0)
			prevOn := true
			curPts := []mgl32.Vec2{}
			for _, pt := range loop {
				x := pt.x
				y := pt.y
				on := pt.on
				if on {
					if !first {
						if !prevOn {
							// close last bezier:
							intersectQuad(curPts[0], curPts[1], mgl32.Vec2{x, y})
							curPts = []mgl32.Vec2{}
						} else {
							intersectLinear(curPts[0], mgl32.Vec2{x, y})
							curPts = []mgl32.Vec2{}
						}
					}
					curPts = append(curPts, mgl32.Vec2{x, y})
				} else {
					if first {
						panic("NYI: rewind the loop to start with an 'on' point")
					}
					if !prevOn {
						// gen midpoint
						midX := (x + prevX) / 2
						midY := (y + prevY) / 2
						// close last:
						intersectQuad(curPts[0], curPts[1], mgl32.Vec2{midX, midY})
						// open next:
						curPts = []mgl32.Vec2{mgl32.Vec2{midX, midY}}
					} // else, we're beginning a new bspline..
					curPts = append(curPts, mgl32.Vec2{x, y})
				}
				if first {
					firstX = x
					firstY = y
					first = false
				}
				prevX = x
				prevY = y
				prevOn = on
			}
			if !prevOn {
				intersectQuad(curPts[0], curPts[1], mgl32.Vec2{firstX, firstY})
				curPts = []mgl32.Vec2{}
			} else {
				intersectLinear(curPts[0], mgl32.Vec2{firstX, firstY})
			}
		}
		xMin := float32(math.MaxFloat32)
		result := false
		for i := 0; i < len(xs); i++ {
			if xs[i] < xMin {
				xMin = xs[i]
				result = interiors[i]
			}
		}
		fmt.Println("q:", q, "in?", result)
		return result
	}
	// to build final mesh:
	// iterate over indices, finding the corresponding triangles in dtTriangles
	// insert those triangles with the appropriate uvs as given by uvs, and mark
	// them as inserted against dtTriangles
	// then, iterate over dtTriangles, for all not-yet-inserted triangles:
	// test whether triangle center is in the glyph; if so, the triangle uvs
	// should be [0 1], otherwise the triangle uvs should be [1 0]
	// to compress, scan glyphMesh for a matching vertex before insertion of a
	// new vertex.
	splineTriangleIs := []int{}
	for i := 0; i < len(indices); i += 3 {
		dtVI0 := srcToDtIs[int(indices[i])]
		dtVI1 := srcToDtIs[int(indices[i+1])]
		dtVI2 := srcToDtIs[int(indices[i+2])]
		triI := -1
		srcIs := []int{}
		concave := false
		for j := 0; j < len(dtTriangles); j += 3 {
			triI = j
			if dtVI0 == dtTriangles[j] {
				if dtVI1 == dtTriangles[j+1] && dtVI2 == dtTriangles[j+2] {
					srcIs = []int{0, 1, 2}
					break
				} else if dtVI1 == dtTriangles[j+2] && dtVI2 == dtTriangles[j+1] {
					srcIs = []int{0, 2, 1}
					concave = true
					break
				}
			} else if dtVI0 == dtTriangles[j+1] {
				if dtVI1 == dtTriangles[j+2] && dtVI2 == dtTriangles[j] {
					srcIs = []int{1, 2, 0}
					break
				} else if dtVI1 == dtTriangles[j] && dtVI2 == dtTriangles[j+2] {
					srcIs = []int{1, 0, 2}
					concave = true
					break
				}
			} else if dtVI0 == dtTriangles[j+2] {
				if dtVI1 == dtTriangles[j] && dtVI2 == dtTriangles[j+1] {
					srcIs = []int{2, 0, 1}
					break
				} else if dtVI1 == dtTriangles[j+1] && dtVI2 == dtTriangles[j] {
					srcIs = []int{2, 1, 0}
					concave = true
					break
				}
			}
		}
		splineTriangleIs = append(splineTriangleIs, triI)
		fmt.Println(srcIs)
		// ...
		for _, n := range srcIs {
			idx := int(indices[i+n])
			dtVIn := srcToDtIs[idx]
			uv := uvs[idx]
			if concave {
				uv += 3
			}
			pos := dtVerts[dtVIn]
			dstVertI := len(glyphMesh.positions) / 2
			glyphMesh.positions = append(glyphMesh.positions, pos[0], pos[1])
			glyphMesh.uvs = append(glyphMesh.uvs, uv)
			glyphMesh.indices = append(glyphMesh.indices, int16(dstVertI))
		}
	}
	for i := 0; i < len(dtTriangles); i += 3 {
		isSpline := false
		for _, j := range splineTriangleIs {
			if j == i {
				isSpline = true
				break
			}
		}
		if isSpline {
			continue
		}
		p0 := dtVerts[dtTriangles[i]]
		p1 := dtVerts[dtTriangles[i+1]]
		p2 := dtVerts[dtTriangles[i+2]]
		mp := p0.Add(p1).Add(p2).Mul(float32(1) / float32(3))
		fmt.Println("mp", mp, "p012", p0, p1, p2)
		uv := int8(uvExterior)
		if pointInGlyph(mp) {
			uv = uvInterior
		}
		// uv = int8((i / 3) % 20)
		ps := []mgl32.Vec2{p0, p1, p2}
		for _, p := range ps {
			dstVertI := -1
			for j := 0; j < len(glyphMesh.positions); j += 2 {
				if glyphMesh.positions[j] == p[0] &&
					glyphMesh.positions[j+1] == p[1] &&
					glyphMesh.uvs[j/2] == uv {
					dstVertI = j / 2
					break
				}
			}
			if dstVertI < 0 {
				dstVertI = len(glyphMesh.positions) / 2
				glyphMesh.positions = append(glyphMesh.positions, p[0], p[1])
				glyphMesh.uvs = append(glyphMesh.uvs, uv)
			}
			glyphMesh.indices = append(glyphMesh.indices, int16(dstVertI))
		}
	}
	fmt.Println(splineTriangleIs)
	fmt.Println(len(glyphMesh.positions), len(glyphMesh.uvs), glyphMesh.indices)
	fmt.Println(glyphMesh.uvs)
	fmt.Println("point in glyph?", pointInGlyph(mgl32.Vec2{testX, testY}))
	glyphMesh.positions = append(glyphMesh.positions,
		testX, testY,
		testX, testY+0.004,
		testX+1, testY)
	glyphMesh.uvs = append(glyphMesh.uvs,
		uvMidConvex, uvMidConvex, uvMidConvex)
	glyphMesh.indices = append(glyphMesh.indices,
		int16(len(glyphMesh.positions)/2-3),
		int16(len(glyphMesh.positions)/2-2),
		int16(len(glyphMesh.positions)/2-1))
}

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

func pointInTriangle(p, a, b, c mgl32.Vec2) bool {
	u, v := getBarycentric(p, a, b, c)
	// all those dot products really accumulate the rounding error!
	return u >= -1e-6 && v >= -1e-6 && u+v-1 < 1e-6
}

func pointInAngle(p, a, b, c mgl32.Vec2) bool {
	u, v := getBarycentric(p, a, b, c)
	return u >= -1e-6 && v >= -1e-6
}

var prog uint32
var vao uint32
var vbos [3]uint32

var transform mgl32.Mat4
var invTransform mgl32.Mat4

func renderInit() {
	var err error
	prog, err = newProgram(vertexShader, fragShader)
	if err != nil {
		panic(err)
	}

	gl.Enable(gl.MULTISAMPLE)
	gl.Enable(gl.BLEND)
	gl.BlendEquationSeparate(gl.FUNC_ADD, gl.FUNC_ADD)
	gl.BlendFuncSeparate(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA, gl.ONE, gl.ZERO)

	gl.UseProgram(prog)

	aspect := float32(1280) / 720
	scale := float32(0.7)
	transform = mgl32.Ortho2D(-aspect*0.2*scale, 1.2*aspect*scale, -0.2*scale, 1.2*scale)
	invTransform = transform.Inv()
	transformU := gl.GetUniformLocation(prog, gl.Str("transform\x00"))
	gl.UniformMatrix4fv(transformU, 1, false, &transform[0])
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.GenBuffers(3, &vbos[0])
	bindBuffers()
}

func bindBuffers() {
	gl.BindBuffer(gl.ARRAY_BUFFER, vbos[0])
	gl.BufferData(gl.ARRAY_BUFFER,
		4*len(glyphMesh.positions), gl.Ptr(glyphMesh.positions),
		gl.STATIC_DRAW)
	posAttrib := uint32(gl.GetAttribLocation(prog, gl.Str("pos\x00")))
	gl.EnableVertexAttribArray(posAttrib)
	gl.VertexAttribPointer(posAttrib, 2, gl.FLOAT, false, 8, gl.PtrOffset(0))

	gl.BindBuffer(gl.ARRAY_BUFFER, vbos[1])
	gl.BufferData(gl.ARRAY_BUFFER,
		len(glyphMesh.uvs), gl.Ptr(glyphMesh.uvs), gl.STATIC_DRAW)
	uvAttrib := uint32(gl.GetAttribLocation(prog, gl.Str("uvI\x00")))
	gl.EnableVertexAttribArray(uvAttrib)
	gl.VertexAttribIPointer(uvAttrib, 1, gl.BYTE, 1, gl.PtrOffset(0))

	gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, vbos[2])
	gl.BufferData(gl.ELEMENT_ARRAY_BUFFER,
		2*len(glyphMesh.indices), gl.Ptr(glyphMesh.indices), gl.STATIC_DRAW)
}

func render() {
	gl.ClearColor(1, 1, 1, 1)
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	gl.DrawElements(gl.TRIANGLES,
		int32(len(glyphMesh.indices)), gl.UNSIGNED_SHORT, gl.PtrOffset(0))
}

func onKey(w *glfw.Window, k glfw.Key, scancode int,
	action glfw.Action, mods glfw.ModifierKey) {
	if k == glfw.KeyEscape {
		panic("esc")
	}
	lowercase := 0
	if 0 == (mods & glfw.ModShift) {
		lowercase = 0x20
	}
	r := rune(int(k) | lowercase)
	loadGlyph(r)
	fmt.Println("rune is now:", string(r))
	bindBuffers()
}

var mouseCoord mgl32.Vec2

func onCursorPos(w *glfw.Window, xpos, ypos float64) {
	mouseCoord = mgl32.Vec2{float32(xpos / 1280), float32(1 - ypos/720)}
	mouseCoord = mouseCoord.Mul(2).Sub(mgl32.Vec2{1, 1})
	mouseCoord = invTransform.Mul4x1(mouseCoord.Vec4(0, 1)).Vec2()
	dTI := len(glyphMesh.positions) - 6
	glyphMesh.positions[dTI] = mouseCoord[0]
	glyphMesh.positions[dTI+1] = mouseCoord[1]
	glyphMesh.positions[dTI+2] = mouseCoord[0]
	glyphMesh.positions[dTI+3] = mouseCoord[1] + 0.004
	glyphMesh.positions[dTI+4] = mouseCoord[0] + 1
	glyphMesh.positions[dTI+5] = mouseCoord[1]
	bindBuffers()
}

func main() {
	if err := glfw.Init(); err != nil {
		log.Fatalln("failed to initialize glfw:", err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.Resizable, glfw.False)
	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)
	glfw.WindowHint(glfw.Samples, 8)
	window, err := glfw.CreateWindow(1280, 720, "go", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()
	err = gl.Init()
	if err != nil {
		panic(err)
	}
	window.SetKeyCallback(onKey)
	window.SetCursorPosCallback(onCursorPos)

	loadFont("SeoulNamsan-Light.ttf")
	loadGlyph('께')

	renderInit()

	for !window.ShouldClose() {
		beginFrame()
		glfw.PollEvents()
		render()
		window.SwapBuffers()
		endFrame()
		title := fmt.Sprintf("frame time - %0.2fms / frame rate - %0.1ffps",
			frameTime, frameRate())
		window.SetTitle(title)
		frameShouldEndTime := frameBeginTime.Add(16 * time.Millisecond)
		if time.Now().Before(frameShouldEndTime) {
			time.Sleep(frameShouldEndTime.Sub(time.Now()))
		}
	}
}

func newProgram(vsGlsl, fsGlsl string) (uint32, error) {
	vertexShader, err := compileShader(vsGlsl, gl.VERTEX_SHADER)
	if err != nil {
		return 0, err
	}

	fragmentShader, err := compileShader(fsGlsl, gl.FRAGMENT_SHADER)
	if err != nil {
		return 0, err
	}

	program := gl.CreateProgram()

	gl.AttachShader(program, vertexShader)
	gl.AttachShader(program, fragmentShader)
	gl.LinkProgram(program)

	var status int32
	gl.GetProgramiv(program, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(program, gl.INFO_LOG_LENGTH, &logLength)

		log := string(make([]byte, int(logLength+1)))
		gl.GetProgramInfoLog(program, logLength, nil, gl.Str(log))

		return 0, errors.New(fmt.Sprintf("failed to link program: %v", log))
	}

	gl.DeleteShader(vertexShader)
	gl.DeleteShader(fragmentShader)

	return program, nil
}

func compileShader(source string, shaderType uint32) (uint32, error) {
	shader := gl.CreateShader(shaderType)

	csource := gl.Str(source)
	gl.ShaderSource(shader, 1, &csource, nil)
	gl.CompileShader(shader)

	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)

		log := string(make([]byte, int(logLength+1)))
		gl.GetShaderInfoLog(shader, logLength, nil, gl.Str(log))

		return 0, fmt.Errorf("failed to compile %v: %v", source, log)
	}

	return shader, nil
}

var vertexShader string = `
#version 330

uniform mat4 transform;

in vec2 pos;
in int uvI;

out vec3 texCoord;

void main() {
	const vec3 uvs[20] = vec3[20](
		vec3(0.0, 0.0, 1.0),
		vec3(0.5, 0.0, 1.0),
		vec3(1.0, 1.0, 1.0),
		vec3(0.0, 0.0, 0.0),
		vec3(0.5, 0.0, 0.0),
		vec3(1.0, 1.0, 0.0),
		vec3(1.0, 0.0, 1.0),
		vec3(0.0, 1.0, 1.0),
		vec3(0.000000, 0.462745, 1.000000),
		vec3(0.835294, 1.000000, 0.000000),
		vec3(1.000000, 0.576471, 0.494118),
		vec3(0.415686, 0.509804, 0.423529),
		vec3(1.000000, 0.007843, 0.615686),
		vec3(0.996078, 0.537255, 0.000000),
		vec3(0.478431, 0.278431, 0.509804),
		vec3(0.494118, 0.176471, 0.823529),
		vec3(0.521569, 0.662745, 0.000000),
		vec3(1.000000, 0.000000, 0.337255),
		vec3(0.643137, 0.141176, 0.000000),
		vec3(0.000000, 0.682353, 0.494118));
	texCoord = vec3(uvs[uvI]);
	gl_Position = transform * vec4(pos, 0.0, 1.0);
}
` + "\x00"

var fragShader string = `
#version 330

in vec3 texCoord;

out vec4 color;

void main() {
	vec2 px = dFdx(texCoord.xy);
	vec2 py = dFdy(texCoord.xy);
	float fx = (2.0*texCoord.x)*px.x - px.y;
	float fy = (2.0*texCoord.x)*py.x - py.y;
	float sd = (texCoord.x*texCoord.x - texCoord.y)/sqrt(fx*fx + fy*fy);
	float alpha = clamp(0.5 - (2.0 * texCoord.z - 1.0) * sd, 0.0, 1.0);
	color = vec4(texCoord, 1.0);
}
` + "\x00"

var colorList []mgl32.Vec3 = []mgl32.Vec3{
	mgl32.Vec3{0x00, 0xFF, 0x00},
	mgl32.Vec3{0x00, 0x00, 0xFF},
	mgl32.Vec3{0xFF, 0x00, 0x00},
	mgl32.Vec3{0x01, 0xFF, 0xFE},
	mgl32.Vec3{0xFF, 0xA6, 0xFE},
	mgl32.Vec3{0xFF, 0xDB, 0x66},
	mgl32.Vec3{0x00, 0x64, 0x01},
	mgl32.Vec3{0x01, 0x00, 0x67},
	mgl32.Vec3{0x95, 0x00, 0x3A},
	mgl32.Vec3{0x00, 0x7D, 0xB5},
	mgl32.Vec3{0xFF, 0x00, 0xF6},
	mgl32.Vec3{0xFF, 0xEE, 0xE8},
	mgl32.Vec3{0x77, 0x4D, 0x00},
	mgl32.Vec3{0x90, 0xFB, 0x92},
	mgl32.Vec3{0x00, 0x76, 0xFF},
	mgl32.Vec3{0xD5, 0xFF, 0x00},
	mgl32.Vec3{0xFF, 0x93, 0x7E},
	mgl32.Vec3{0x6A, 0x82, 0x6C},
	mgl32.Vec3{0xFF, 0x02, 0x9D},
	mgl32.Vec3{0xFE, 0x89, 0x00},
	mgl32.Vec3{0x7A, 0x47, 0x82},
	mgl32.Vec3{0x7E, 0x2D, 0xD2},
	mgl32.Vec3{0x85, 0xA9, 0x00},
	mgl32.Vec3{0xFF, 0x00, 0x56},
	mgl32.Vec3{0xA4, 0x24, 0x00},
	mgl32.Vec3{0x00, 0xAE, 0x7E},
	mgl32.Vec3{0x68, 0x3D, 0x3B},
	mgl32.Vec3{0xBD, 0xC6, 0xFF},
	mgl32.Vec3{0x26, 0x34, 0x00},
	mgl32.Vec3{0xBD, 0xD3, 0x93},
	mgl32.Vec3{0x00, 0xB9, 0x17},
	mgl32.Vec3{0x9E, 0x00, 0x8E},
	mgl32.Vec3{0x00, 0x15, 0x44},
	mgl32.Vec3{0xC2, 0x8C, 0x9F},
	mgl32.Vec3{0xFF, 0x74, 0xA3},
	mgl32.Vec3{0x01, 0xD0, 0xFF},
	mgl32.Vec3{0x00, 0x47, 0x54},
	mgl32.Vec3{0xE5, 0x6F, 0xFE},
	mgl32.Vec3{0x78, 0x82, 0x31},
	mgl32.Vec3{0x0E, 0x4C, 0xA1},
	mgl32.Vec3{0x91, 0xD0, 0xCB},
	mgl32.Vec3{0xBE, 0x99, 0x70},
	mgl32.Vec3{0x96, 0x8A, 0xE8},
	mgl32.Vec3{0xBB, 0x88, 0x00},
	mgl32.Vec3{0x43, 0x00, 0x2C},
	mgl32.Vec3{0xDE, 0xFF, 0x74},
	mgl32.Vec3{0x00, 0xFF, 0xC6},
	mgl32.Vec3{0xFF, 0xE5, 0x02},
	mgl32.Vec3{0x62, 0x0E, 0x00},
	mgl32.Vec3{0x00, 0x8F, 0x9C},
	mgl32.Vec3{0x98, 0xFF, 0x52},
	mgl32.Vec3{0x75, 0x44, 0xB1},
	mgl32.Vec3{0xB5, 0x00, 0xFF},
	mgl32.Vec3{0x00, 0xFF, 0x78},
	mgl32.Vec3{0xFF, 0x6E, 0x41},
	mgl32.Vec3{0x00, 0x5F, 0x39},
	mgl32.Vec3{0x6B, 0x68, 0x82},
	mgl32.Vec3{0x5F, 0xAD, 0x4E},
	mgl32.Vec3{0xA7, 0x57, 0x40},
	mgl32.Vec3{0xA5, 0xFF, 0xD2},
	mgl32.Vec3{0xFF, 0xB1, 0x67},
	mgl32.Vec3{0x00, 0x9B, 0xFF},
	mgl32.Vec3{0xE8, 0x5E, 0xBE},
}