#!/usr/bin/env node

const fs = require('fs')
const path = require('path')

const SEGMENTS = 64
const OUTPUT = path.resolve(__dirname, '..', 'meshes', 'visual')

async function loadKernel () {
  const moduleFactory = (await import('manifold-3d')).default
  const kernel = await moduleFactory()
  kernel.setup()
  return kernel
}

function createFactories (Manifold) {
  const box = (size, position = [0, 0, 0]) =>
    Manifold.cube(size, true).translate(position)

  const cylZ = (radius, height, position = [0, 0, 0]) =>
    Manifold.cylinder(height, radius, radius, SEGMENTS, true).translate(position)

  const cylY = (radius, height, position = [0, 0, 0]) =>
    Manifold.cylinder(height, radius, radius, SEGMENTS, true)
      .rotate([90, 0, 0]).translate(position)

  const cylX = (radius, height, position = [0, 0, 0]) =>
    Manifold.cylinder(height, radius, radius, SEGMENTS, true)
      .rotate([0, 90, 0]).translate(position)

  const union = (...solids) => Manifold.union(solids)
  const subtract = (body, ...cutters) => {
    let result = body
    for (const cutter of cutters) result = result.subtract(cutter)
    return result
  }

  function basePlate () {
    let plate = cylZ(92, 8, [0, 0, 4])
    const holes = [
      cylZ(12, 12, [0, 0, 4]),
      ...[45, 135, 225, 315].map(deg => {
        const a = deg * Math.PI / 180
        return cylZ(2.2, 12, [70 * Math.cos(a), 70 * Math.sin(a), 4])
      })
    ]
    plate = subtract(plate, ...holes)
    return plate
  }

  function rotatingBase () {
    const disk = cylZ(55, 12, [0, 0, 0])
    const leftTower = box([52, 6, 82], [0, 35, 41])
    const rightTower = box([52, 6, 82], [0, -35, 41])
    const servoShelf = box([52, 70, 6], [0, 0, 8])
    const gusset1 = box([20, 12, 28], [-16, 27, 17])
    const gusset2 = box([20, 12, 28], [-16, -27, 17])
    const shoulderBoss1 = cylY(12, 6, [0, 35, 75])
    const shoulderBoss2 = cylY(12, 6, [0, -35, 75])
    let body = union(
      disk, leftTower, rightTower, servoShelf, gusset1, gusset2,
      shoulderBoss1, shoulderBoss2
    )
    body = subtract(
      body,
      cylY(3.2, 90, [0, 0, 75]),
      box([42.0, 21.0, 44.0], [0, 0, 36]),
      cylZ(8, 30, [0, 0, 0])
    )
    return body
  }

  function twinPlateLink (length, plateSeparation, height, plateThickness) {
    const railY = plateSeparation / 2
    const rail1 = box(
      [length, plateThickness, height], [length / 2, railY, 0]
    )
    const rail2 = box(
      [length, plateThickness, height], [length / 2, -railY, 0]
    )
    const bosses = [
      cylY(height * 0.60, plateThickness, [0, railY, 0]),
      cylY(height * 0.60, plateThickness, [0, -railY, 0]),
      cylY(height * 0.60, plateThickness, [length, railY, 0]),
      cylY(height * 0.60, plateThickness, [length, -railY, 0])
    ]
    const spacers = [
      box(
        [8, plateSeparation + plateThickness, height * 0.45],
        [length * 0.30, 0, 0]
      ),
      box(
        [8, plateSeparation + plateThickness, height * 0.45],
        [length * 0.70, 0, 0]
      )
    ]
    let link = union(rail1, rail2, ...bosses, ...spacers)
    link = subtract(
      link,
      cylY(3.2, plateSeparation + 20, [0, 0, 0]),
      cylY(3.2, plateSeparation + 20, [length, 0, 0]),
      box(
        [length - 28, plateSeparation - 4, height * 0.55],
        [length / 2, 0, 0]
      )
    )
    return link
  }

  function upperArm () {
    return twinPlateLink(145, 32, 30, 5)
  }

  function forearm () {
    return twinPlateLink(125, 27, 26, 4.5)
  }

  function wristRollHousing () {
    const body = union(
      box([40, 35, 30], [20, 0, 0]),
      cylX(18, 5, [0, 0, 0])
    )
    return subtract(
      body,
      box([28, 13.4, 36], [21, 0, 7]),
      cylX(3.0, 50, [0, 0, 0]),
      box([18, 24, 18], [39, 0, 0])
    )
  }

  function wristPitchBracket () {
    const side1 = box([35, 5, 30], [17.5, 14.5, 0])
    const side2 = box([35, 5, 30], [17.5, -14.5, 0])
    const bridge = box([10, 34, 30], [5, 0, 0])
    const endBosses = [
      cylY(16, 5, [35, 14.5, 0]),
      cylY(16, 5, [35, -14.5, 0])
    ]
    return subtract(
      union(side1, side2, bridge, ...endBosses),
      cylY(3.0, 50, [0, 0, 0]),
      cylY(3.0, 50, [35, 0, 0]),
      box([23.0, 13.4, 28.5], [18, 0, 2])
    )
  }

  function toolFlange () {
    const shaft = cylX(16, 35, [17.5, 0, 0])
    const mountingFace = box([4, 44, 30], [33, 0, 0])
    const wireHole = cylX(4.5, 42, [17.5, 0, 0])
    const screwHoles = [
      cylX(1.7, 10, [33, 14, 8]),
      cylX(1.7, 10, [33, -14, 8]),
      cylX(1.7, 10, [33, 14, -8]),
      cylX(1.7, 10, [33, -14, -8])
    ]
    return subtract(union(shaft, mountingFace), wireHole, ...screwHoles)
  }

  function gripperPalm () {
    const palm = box([20, 80, 24], [10, 0, 0])
    const frontRail = box([12, 74, 10], [24, 0, 0])
    return subtract(
      union(palm, frontRail),
      box([24, 24.2, 14.0], [8, 0, 7]),
      cylX(2.2, 30, [10, 31, 0]),
      cylX(2.2, 30, [10, -31, 0]),
      box([32, 6, 6], [18, 0, 0])
    )
  }

  function finger () {
    const rail = box([45, 10, 14], [22.5, 0, 0])
    const pad = box([13, 14, 27], [39.5, 0, -6.5])
    const dovetail = box([12, 14, 8], [3, 0, 0])
    return subtract(
      union(rail, pad, dovetail),
      cylZ(1.7, 24, [6, 0, 0]),
      cylZ(1.7, 24, [18, 0, 0])
    )
  }

  function cameraMount () {
    const backPlate = box([4, 34, 28], [0, 0, 0])
    const topHood = box([24, 34, 3], [10, 0, 13])
    const bottomHood = box([16, 34, 3], [6, 0, -13])
    const side1 = box([18, 3, 26], [7, 15.5, 0])
    const side2 = box([18, 3, 26], [7, -15.5, 0])
    return subtract(
      union(backPlate, topHood, bottomHood, side1, side2),
      cylX(4.2, 12, [0, 0, 0]),
      cylX(1.3, 12, [0, 10.8, 6.25]),
      cylX(1.3, 12, [0, -10.8, 6.25]),
      cylX(1.3, 12, [0, 10.8, -6.25]),
      cylX(1.3, 12, [0, -10.8, -6.25]),
      box([12, 18, 6], [0, 0, -12])
    )
  }

  return {
    base_plate: basePlate,
    rotating_base: rotatingBase,
    upper_arm: upperArm,
    forearm: forearm,
    wrist_roll_housing: wristRollHousing,
    wrist_pitch_bracket: wristPitchBracket,
    tool_flange: toolFlange,
    gripper_palm: gripperPalm,
    finger_left: finger,
    finger_right: finger,
    camera_mount: cameraMount
  }
}

function vector (mesh, index) {
  const offset = mesh.numProp * index
  return [
    mesh.vertProperties[offset],
    mesh.vertProperties[offset + 1],
    mesh.vertProperties[offset + 2]
  ]
}

function normal (a, b, c) {
  const ux = b[0] - a[0]
  const uy = b[1] - a[1]
  const uz = b[2] - a[2]
  const vx = c[0] - a[0]
  const vy = c[1] - a[1]
  const vz = c[2] - a[2]
  let nx = uy * vz - uz * vy
  let ny = uz * vx - ux * vz
  let nz = ux * vy - uy * vx
  const length = Math.hypot(nx, ny, nz) || 1
  nx /= length
  ny /= length
  nz /= length
  return [nx, ny, nz]
}

function writeStl (name, solid) {
  const mesh = solid.getMesh()
  const lines = [`solid ${name}`]
  for (let triangle = 0; triangle < mesh.numTri; triangle++) {
    const start = triangle * 3
    const a = vector(mesh, mesh.triVerts[start])
    const b = vector(mesh, mesh.triVerts[start + 1])
    const c = vector(mesh, mesh.triVerts[start + 2])
    const n = normal(a, b, c)
    lines.push(`  facet normal ${n[0]} ${n[1]} ${n[2]}`)
    lines.push('    outer loop')
    for (const v of [a, b, c]) {
      lines.push(`      vertex ${v[0]} ${v[1]} ${v[2]}`)
    }
    lines.push('    endloop')
    lines.push('  endfacet')
  }
  lines.push(`endsolid ${name}`)
  const file = path.join(OUTPUT, `${name}.stl`)
  fs.writeFileSync(file, `${lines.join('\n')}\n`)
  console.log(
    `generated ${path.relative(process.cwd(), file)} ` +
    `(${mesh.numTri} triangles)`
  )
}

async function main () {
  const { Manifold } = await loadKernel()
  const parts = createFactories(Manifold)
  fs.mkdirSync(OUTPUT, { recursive: true })
  for (const [name, factory] of Object.entries(parts)) {
    writeStl(name, factory())
  }
}

if (require.main === module) {
  main().catch(error => {
    console.error(error)
    process.exitCode = 1
  })
}

module.exports = { main }
