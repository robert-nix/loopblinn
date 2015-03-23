// portions copied from https://github.com/go-gl/examples
package main

import (
	"errors"
	"fmt"
	"io/ioutil"
	"log"
	"math"
	"os"
	"runtime"
	"time"

	"github.com/davecheney/profile"
	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.1/glfw"
	"github.com/go-gl/mathgl/mgl32"

	"code.google.com/p/freetype-go/freetype/truetype"

	"github.com/Mischanix/loopblinn/cdt"
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
			if on && !foundStartPoint {
				foundStartPoint = true
				loopStartI = loopI
			}
			loop = append(loop, point{x, y, on})
			loopI++
			i++
		}
		loop = append(loop[loopStartI:], loop[:loopStartI]...)
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
	// define points and bezier triangles:
	glyphMesh = GlyphMesh{}
	positions := make([]float32, 0)
	uvs := make([]int8, 0)
	indices := make([]int16, 0)
	lines := []int16{}
	addVert := func(x, y float32, uv int8) {
		positions = append(positions, x, y)
		uvs = append(uvs, uv)
	}
	n := int16(0)
	addIndex := func(idx int16) {
		indices = append(indices, idx)
		n++
	}
	addLine := func(a, b int16) {
		lines = append(lines, a, b)
	}
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
			x = pt.x
			y = pt.y
			on := pt.on
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
			// close loop by finalizing the last bezier:
			addVert(firstX, firstY, uvEndConvex)
			addIndex(n)
		} else {
			// close loop with a line, if necessary:
			if x != firstX || y != firstY {
				addLine(n, firstI)
			}
			n++
		}
	}

	// tessellate the glyph with CDT:
	dt := cdt.NewTriangulation(xMin, xMax, yMin, yMax, len(positions)/2)
	srcToDtIs := []int{}
	for i := 0; i < len(positions); i += 2 {
		dtI := dt.AddPoint(positions[i+0], positions[i+1])
		srcToDtIs = append(srcToDtIs, dtI)
	}
	for i := 0; i < len(indices); i += 3 {
		dt.AddEdge(srcToDtIs[int(indices[i+0])], srcToDtIs[int(indices[i+1])])
		dt.AddEdge(srcToDtIs[int(indices[i+1])], srcToDtIs[int(indices[i+2])])
		dt.AddEdge(srcToDtIs[int(indices[i+2])], srcToDtIs[int(indices[i+0])])
	}
	for i := 0; i < len(lines); i += 2 {
		dt.AddEdge(srcToDtIs[int(lines[i])], srcToDtIs[int(lines[i+1])])
	}

	// determine whether a given point is in or outside the glyph shape
	pointInGlyph := func(q mgl32.Vec2) bool {
		xs := []float32{}
		mpxs := []float32{} // used as a tiebreaker
		interiors := []bool{}
		intersectQuad := func(p0, p1, p2 mgl32.Vec2) {
			a := p0[1]
			b := p1[1]
			c := p2[1]
			y := q[1]
			ts := [2]float32{-1, -1}
			if a == c && b == y {
				// undefined
				return
			} else if a == b && c == y {
				ts[0] = 1
				ts[1] = -1
			} else if a == y && b == c {
				ts[0] = 2
				ts[1] = 0
			} else if denom := (a - b) + (c - b); !mgl32.FloatEqual(denom, 0) {
				det := (b-y)*(b-y) + y*(c+a-y) - a*c
				if det < 0 {
					// paraboloid never intersects:
					return
				}
				root := float32(math.Sqrt(float64(det)))
				ts[0] = (root + a - b) / denom
				ts[1] = (-root + a - b) / denom
			} else if !mgl32.FloatEqual(b-c, 0) {
				// quadratic has only one intersection with x = k:
				ts[0] = (2*b - c - y) / (2 * (b - c))
			} else {
				// shouldn't reach here
				return
			}
			for _, t := range ts {
				if t < 0 || t > 1 {
					continue
				}
				// get x for t:
				x := (1-t)*((1-t)*p0[0]+t*p1[0]) + t*((1-t)*p1[0]+t*p2[0])
				mpx := 0.25*p0[0] + 0.5*p1[0] + 0.25*p2[0]
				if x < q[0] {
					continue
				}
				// to determine interiority, we need the vector derivative of
				// the bezier at t:
				dB := p1.Sub(p0).Mul(2 * (1 - t)).Add(p2.Sub(p1).Mul(2 * t))
				if dB[1] > 0 {
					// q is exterior
					xs = append(xs, x)
					mpxs = append(mpxs, mpx)
					interiors = append(interiors, false)
				} else if dB[1] < 0 {
					// q is interior
					xs = append(xs, x)
					mpxs = append(mpxs, mpx)
					interiors = append(interiors, true)
				} // otherwise, y = 0 => doesn't matter
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
				mpx := 0.5*p0[0] + 0.5*p1[0]
				xs = append(xs, x)
				mpxs = append(mpxs, mpx)
				if p0[1] < p1[1] {
					// q is exterior
					interiors = append(interiors, false)
				} else {
					// q is interior
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
		mpxMin := float32(math.MaxFloat32)
		result := false
		for i := 0; i < len(xs); i++ {
			if xs[i] < xMin || (xs[i] == xMin && mpxs[i] < mpxMin) {
				xMin = xs[i]
				mpxMin = mpxs[i]
				result = interiors[i]
			}
		}
		return result
	}
	// to build final mesh:
	// iterate over indices, finding the corresponding triangles in dt.Triangles
	// insert those triangles with the appropriate uvs as given by uvs, and mark
	// them as inserted against dt.Triangles
	// then, iterate over dt.Triangles, for all not-yet-inserted triangles: test
	// whether triangle center is in the glyph; if so, the triangle uvs should
	// be [0 1], otherwise the triangle uvs should be [1 0] to compress, scan
	// glyphMesh for a matching vertex before insertion of a new vertex.
	splineTriangleIs := []int{}
	for i := 0; i < len(indices); i += 3 {
		dtVI0 := srcToDtIs[int(indices[i])]
		dtVI1 := srcToDtIs[int(indices[i+1])]
		dtVI2 := srcToDtIs[int(indices[i+2])]
		triI := -1
		srcIs := []int{}
		concave := false
		for j := 0; j < dt.TriangleI; j += 3 {
			triI = j
			if dtVI0 == dt.Triangles[j] {
				if dtVI1 == dt.Triangles[j+1] && dtVI2 == dt.Triangles[j+2] {
					srcIs = []int{0, 1, 2}
					break
				} else if dtVI1 == dt.Triangles[j+2] && dtVI2 == dt.Triangles[j+1] {
					srcIs = []int{0, 2, 1}
					concave = true
					break
				}
			} else if dtVI0 == dt.Triangles[j+1] {
				if dtVI1 == dt.Triangles[j+2] && dtVI2 == dt.Triangles[j] {
					srcIs = []int{1, 2, 0}
					break
				} else if dtVI1 == dt.Triangles[j] && dtVI2 == dt.Triangles[j+2] {
					srcIs = []int{1, 0, 2}
					concave = true
					break
				}
			} else if dtVI0 == dt.Triangles[j+2] {
				if dtVI1 == dt.Triangles[j] && dtVI2 == dt.Triangles[j+1] {
					srcIs = []int{2, 0, 1}
					break
				} else if dtVI1 == dt.Triangles[j+1] && dtVI2 == dt.Triangles[j] {
					srcIs = []int{2, 1, 0}
					concave = true
					break
				}
			}
		}
		splineTriangleIs = append(splineTriangleIs, triI)
		// ...
		for _, n := range srcIs {
			idx := int(indices[i+n])
			dtVIn := srcToDtIs[idx]
			uv := uvs[idx]
			if concave {
				uv += 3
			}
			pos := dt.Verts[dtVIn]
			dstVertI := len(glyphMesh.positions) / 2
			glyphMesh.positions = append(glyphMesh.positions, pos[0], pos[1])
			glyphMesh.uvs = append(glyphMesh.uvs, uv)
			glyphMesh.indices = append(glyphMesh.indices, int16(dstVertI))
		}
	}
	for i := 0; i < dt.TriangleI; i += 3 {
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
		p0 := dt.Verts[dt.Triangles[i]]
		p1 := dt.Verts[dt.Triangles[i+1]]
		p2 := dt.Verts[dt.Triangles[i+2]]
		mp := p0.Add(p1).Add(p2).Mul(float32(1) / float32(3))
		uv := int8(uvExterior)
		if pointInGlyph(mp) {
			uv = uvInterior
		}
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
	if action == glfw.Press {
		r := rune(int(k) | lowercase)
		loadGlyph(r)
		fmt.Println("rune is now:", string(r))
		bindBuffers()
	}
}

var mouseCoord mgl32.Vec2

func onCursorPos(w *glfw.Window, xpos, ypos float64) {
	mouseCoord = mgl32.Vec2{float32(xpos / 1280), float32(1 - ypos/720)}
	mouseCoord = mouseCoord.Mul(2).Sub(mgl32.Vec2{1, 1})
	mouseCoord = invTransform.Mul4x1(mouseCoord.Vec4(0, 1)).Vec2()
	fmt.Println(mouseCoord)
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
	startTime := glfw.GetTime()
	prof := profile.Start(profile.CPUProfile)
	for i := 0; i < 100; i++ {
		loadGlyph('께')
		loadGlyph('c')
		loadGlyph('h')
		loadGlyph('g')
		loadGlyph('r')
		loadGlyph('R')
		loadGlyph('o')
		loadGlyph('3')
		loadGlyph('4')
	}
	prof.Stop()
	fmt.Printf("loadGlyphs took %fms\n", 1e3*(glfw.GetTime()-startTime))

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
	const vec3 uvs[8] = vec3[8](
		vec3(0.0, 0.0, 1.0),
		vec3(0.5, 0.0, 1.0),
		vec3(1.0, 1.0, 1.0),
		vec3(0.0, 0.0, 0.0),
		vec3(0.5, 0.0, 0.0),
		vec3(1.0, 1.0, 0.0),
		vec3(1.0, 0.0, 1.0),
		vec3(0.0, 1.0, 1.0));
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
	color = vec4(0.0, 0.0, 0.0, alpha);
}
` + "\x00"
