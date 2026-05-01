/**
 * WorldCanvas — live 2D world-frame canvas
 *
 * Draws three toggleable trails + lidar point cloud + robot pose.
 * Coordinate system: world-frame mm, (0,0) bottom-left, X right, Y up.
 * Scale auto-grows to fit all data; never shrinks (preserves history).
 */
import { useRef, useEffect, useState, useCallback } from 'react'
import { useRobotStore } from '../store/robotStore'

const CANVAS_PX = 420          // square canvas size in pixels
const PAD_MM    = 200           // padding around data extent in mm
const LIDAR_THROTTLE_MS = 200  // ~5 Hz lidar render cap

interface Trails {
  odom:  boolean
  gps:   boolean
  fused: boolean
  lidar: boolean
}

export function WorldCanvas() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const fusedPose      = useRobotStore((s) => s.fusedPose)
  const fusedTrail     = useRobotStore((s) => s.fusedPoseTrail)
  const odomTrail      = useRobotStore((s) => s.odometryTrail)
  const gpsStatus      = useRobotStore((s) => s.gpsStatus)
  const lidarPoints    = useRobotStore((s) => s.lidarPoints)

  // Auto-scale state: grows to encompass all data, never shrinks.
  const scaleRef = useRef<{ minX: number; maxX: number; minY: number; maxY: number } | null>(null)

  const [trails, setTrails] = useState<Trails>({ odom: true, gps: true, fused: true, lidar: true })
  const lastLidarRenderRef = useRef<number>(0)

  const toggleTrail = useCallback((key: keyof Trails) => {
    setTrails((t) => ({ ...t, [key]: !t[key] }))
  }, [])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Collect all world-frame points to compute extent.
    const allPts: Array<[number, number]> = []
    if (trails.fused)  allPts.push(...fusedTrail)
    if (trails.odom)   allPts.push(...odomTrail)
    if (fusedPose)     allPts.push([fusedPose.x, fusedPose.y])
    if (trails.gps && gpsStatus?.is_detected) allPts.push([gpsStatus.x, gpsStatus.y])
    if (trails.lidar && lidarPoints) {
      for (let i = 0; i < lidarPoints.xs.length; i++) {
        allPts.push([lidarPoints.xs[i], lidarPoints.ys[i]])
      }
    }

    if (allPts.length > 0) {
      const xs = allPts.map((p) => p[0])
      const ys = allPts.map((p) => p[1])
      const newMin = { minX: Math.min(...xs) - PAD_MM, maxX: Math.max(...xs) + PAD_MM,
                       minY: Math.min(...ys) - PAD_MM, maxY: Math.max(...ys) + PAD_MM }
      if (!scaleRef.current) {
        scaleRef.current = newMin
      } else {
        scaleRef.current = {
          minX: Math.min(scaleRef.current.minX, newMin.minX),
          maxX: Math.max(scaleRef.current.maxX, newMin.maxX),
          minY: Math.min(scaleRef.current.minY, newMin.minY),
          maxY: Math.max(scaleRef.current.maxY, newMin.maxY),
        }
      }
    }

    const ext = scaleRef.current ?? { minX: -500, maxX: 500, minY: -500, maxY: 500 }
    const rangeX = Math.max(ext.maxX - ext.minX, 1)
    const rangeY = Math.max(ext.maxY - ext.minY, 1)
    const scale = CANVAS_PX / Math.max(rangeX, rangeY)

    // World → canvas (Y flipped: canvas Y=0 is top, world Y=0 is bottom)
    const toC = (wx: number, wy: number): [number, number] => [
      (wx - ext.minX) * scale,
      CANVAS_PX - (wy - ext.minY) * scale,
    ]

    // Clear
    ctx.clearRect(0, 0, CANVAS_PX, CANVAS_PX)
    ctx.fillStyle = 'rgba(0,0,0,0.6)'
    ctx.fillRect(0, 0, CANVAS_PX, CANVAS_PX)

    // Grid
    const gridMm = rangeX > 4000 ? 1000 : rangeX > 1000 ? 500 : 200
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'
    ctx.lineWidth = 0.5
    const startX = Math.floor(ext.minX / gridMm) * gridMm
    const startY = Math.floor(ext.minY / gridMm) * gridMm
    for (let gx = startX; gx <= ext.maxX; gx += gridMm) {
      const [cx] = toC(gx, 0)
      ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, CANVAS_PX); ctx.stroke()
    }
    for (let gy = startY; gy <= ext.maxY; gy += gridMm) {
      const [, cy] = toC(0, gy)
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(CANVAS_PX, cy); ctx.stroke()
    }

    // Scale label
    ctx.fillStyle = 'rgba(255,255,255,0.5)'
    ctx.font = '10px monospace'
    const label = gridMm >= 1000 ? `${gridMm / 1000} m` : `${gridMm} mm`
    ctx.fillText(`grid: ${label}`, 6, CANVAS_PX - 6)

    // Odometry trail (cyan)
    if (trails.odom && odomTrail.length > 1) {
      ctx.beginPath()
      ctx.strokeStyle = 'rgba(0,220,255,0.5)'
      ctx.lineWidth = 1
      odomTrail.forEach(([wx, wy], i) => {
        const [cx, cy] = toC(wx, wy)
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
      })
      ctx.stroke()
    }

    // Fused pose trail (green)
    if (trails.fused && fusedTrail.length > 1) {
      ctx.beginPath()
      ctx.strokeStyle = 'rgba(80,255,120,0.7)'
      ctx.lineWidth = 1.5
      fusedTrail.forEach(([wx, wy], i) => {
        const [cx, cy] = toC(wx, wy)
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
      })
      ctx.stroke()
    }

    // GPS dots (yellow)
    if (trails.gps && gpsStatus?.is_detected) {
      const [cx, cy] = toC(gpsStatus.x, gpsStatus.y)
      ctx.beginPath()
      ctx.arc(cx, cy, 4, 0, Math.PI * 2)
      ctx.fillStyle = 'rgba(255,220,0,0.9)'
      ctx.fill()
    }

    // Lidar cloud (red dots, throttled)
    const now = Date.now()
    if (trails.lidar && lidarPoints && (now - lastLidarRenderRef.current) >= LIDAR_THROTTLE_MS) {
      lastLidarRenderRef.current = now
      ctx.fillStyle = 'rgba(255,80,80,0.6)'
      for (let i = 0; i < lidarPoints.xs.length; i++) {
        const [cx, cy] = toC(lidarPoints.xs[i], lidarPoints.ys[i])
        ctx.beginPath(); ctx.arc(cx, cy, 1.5, 0, Math.PI * 2); ctx.fill()
      }
    }

    // Robot at fused pose
    if (fusedPose) {
      const [rx, ry] = toC(fusedPose.x, fusedPose.y)
      const headLen = 14
      const hx = rx + headLen * Math.cos(fusedPose.theta)
      const hy = ry - headLen * Math.sin(fusedPose.theta)

      // Heading arrow
      ctx.beginPath()
      ctx.moveTo(rx, ry); ctx.lineTo(hx, hy)
      ctx.strokeStyle = 'rgba(255,255,255,0.9)'
      ctx.lineWidth = 2
      ctx.stroke()

      // Robot body
      ctx.beginPath()
      ctx.arc(rx, ry, 7, 0, Math.PI * 2)
      ctx.fillStyle = 'rgba(255,255,255,0.9)'
      ctx.fill()
    }
  }, [fusedPose, fusedTrail, odomTrail, gpsStatus, lidarPoints, trails])

  const btnClass = (on: boolean) =>
    `px-2 py-0.5 rounded text-xs font-medium transition-colors ${
      on ? 'bg-white/20 text-white' : 'bg-white/5 text-white/40'
    }`

  return (
    <div className="space-y-2">
      <div className="flex flex-wrap gap-1.5 items-center">
        <span className="text-xs text-white/50 mr-1">Show:</span>
        <button className={btnClass(trails.odom)}  onClick={() => toggleTrail('odom')}>
          <span className="inline-block w-2 h-2 rounded-full bg-cyan-400 mr-1" />Odometry
        </button>
        <button className={btnClass(trails.gps)}   onClick={() => toggleTrail('gps')}>
          <span className="inline-block w-2 h-2 rounded-full bg-yellow-400 mr-1" />GPS
        </button>
        <button className={btnClass(trails.fused)} onClick={() => toggleTrail('fused')}>
          <span className="inline-block w-2 h-2 rounded-full bg-green-400 mr-1" />Fused
        </button>
        <button className={btnClass(trails.lidar)} onClick={() => toggleTrail('lidar')}>
          <span className="inline-block w-2 h-2 rounded-full bg-red-400 mr-1" />Lidar
        </button>
      </div>
      <canvas
        ref={canvasRef}
        width={CANVAS_PX}
        height={CANVAS_PX}
        className="rounded-xl w-full"
        style={{ imageRendering: 'pixelated' }}
      />
    </div>
  )
}
