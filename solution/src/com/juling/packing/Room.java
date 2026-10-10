package com.juling.packing;

import java.util.ArrayList;
import java.util.List;

/**
 * 房间：轮廓多边形、门、以及门造成的禁放区。
 *
 * 门的建模（题目第 5 条）：
 *   - 任何门的门洞都不能被物品遮挡，人要能进出；
 *   - 外开门门扇向外开，室内仅保留一条通行净空（深度 {@link #DOOR_PASS_DEPTH}）；
 *   - 内开门门扇向内旋转 90 度，扫过半径为门宽 N 的四分之一圆，
 *     题目将其简化为 N x N 的方形禁放区，本程序按 N x N 矩形建模（比扇形更保守）。
 * 禁放区是一个与墙对齐的 OBB：沿墙方向长度 N，垂直墙方向向室内延伸。
 */
public final class Room {
    /** 外开门（及所有门洞）在室内保留的通行净空深度，毫米 */
    public static final double DOOR_PASS_DEPTH = 600.0;
    /** 矩形包含校验的采样步长，毫米 */
    public static final double SAMPLE_STEP = 100.0;

    public final Vec[] boundary;
    public final Vec doorA;
    public final Vec doorB;
    public final boolean openInward;
    public final double doorWidth;
    public final Vec doorDir;            // 沿墙单位向量（doorA -> doorB）
    public final Vec doorInNormal;       // 门处指向室内的单位法向
    public final List<Obb> blockedZones; // 门禁放区

    public Room(Vec[] boundary, Vec doorA, Vec doorB, boolean openInward) {
        this.boundary = boundary;
        this.doorA = doorA;
        this.doorB = doorB;
        this.openInward = openInward;
        this.doorDir = doorB.sub(doorA).normalize();
        this.doorInNormal = Geometry.inwardNormal(doorA, doorB, boundary);
        this.doorWidth = doorA.distanceTo(doorB);
        this.blockedZones = new ArrayList<Obb>();
        buildDoorZone();
    }

    private void buildDoorZone() {
        Vec mid = doorA.add(doorB).mul(0.5);
        double depth = openInward ? doorWidth : DOOR_PASS_DEPTH;
        // 中心向室内推进 depth/2；沿墙半长 = N/2，进深半长 = depth/2
        Vec center = mid.add(doorInNormal.mul(depth / 2.0));
        Obb zone = new Obb(center, doorDir, doorInNormal, doorWidth / 2.0, depth / 2.0);
        blockedZones.add(zone);
    }

    /** 墙段列表（闭合） */
    public Vec[][] walls() {
        int n = boundary.length;
        Vec[][] walls = new Vec[n][];
        for (int i = 0; i < n; i++) {
            walls[i] = new Vec[] { boundary[i], boundary[(i + 1) % n] };
        }
        return walls;
    }

    /**
     * 轮廓所有边的方向角集合（度，归一化到 [0,180)，按 1.5 度容差合并）。
     * 题目要求物品必须与某条轮廓边平行或垂直：物品沿某边“平行”摆放取该角度即可；
     * “垂直”摆放通过让物品的 width 边贴该墙实现（见 Solver），无需额外角度。
     */
    public List<Double> wallAngles() {
        List<Double> angles = new ArrayList<Double>();
        for (Vec[] w : walls()) {
            double len = w[1].sub(w[0]).length();
            if (len < 1.0) continue;
            double a = Vec.lineAngle(w[1].sub(w[0]));
            boolean exists = false;
            for (Double e : angles) {
                if (Vec.angleDiff(e, a) < 1.5) { exists = true; break; }
            }
            if (!exists) angles.add(a);
        }
        return angles;
    }

    /** 物品矩形是否可以放在这个位置（完全在轮廓内、不压门区） */
    public boolean fits(Obb body) {
        if (!Geometry.rectInsidePolygon(body, boundary, SAMPLE_STEP)) return false;
        for (Obb zone : blockedZones) {
            if (body.intersects(zone, Geometry.EPS)) return false;
        }
        return true;
    }

    /**
     * 计算矩形贴墙的总边长（毫米）：矩形的边与某段墙平行、共线且投影重叠即计入。
     */
    public double wallContactLength(Obb body) {
        double total = 0;
        Vec[][] be = body.edges();
        for (Vec[] e : be) {
            Vec eDir = e[1].sub(e[0]).normalize();
            Vec eMid = e[0].add(e[1]).mul(0.5);
            double best = 0;
            for (Vec[] w : walls()) {
                Vec wDir = w[1].sub(w[0]).normalize();
                if (Vec.angleDiff(Vec.lineAngle(eDir), Vec.lineAngle(wDir)) > 2.0) continue;
                Vec wMid = w[0].add(w[1]).mul(0.5);
                // 两边中心线距离（中点连线在墙法向上的投影）
                Vec wn = new Vec(-wDir.y, wDir.x);
                double dist = Math.abs(eMid.sub(wMid).dot(wn));
                if (dist > 15.0) continue;
                double overlap = Geometry.collinearOverlap(e[0], e[1], w[0], w[1], wDir);
                best = Math.max(best, overlap);
            }
            total += best;
        }
        return total;
    }

    /**
     * 判断矩形的某条边是否整边贴在某段墙上（用于判断冰箱背面是否靠墙）。
     * @param edge 矩形的一条边（两端点）
     */
    public boolean edgeAgainstWall(Vec[] edge) {
        Vec eDir = edge[1].sub(edge[0]).normalize();
        Vec eMid = edge[0].add(edge[1]).mul(0.5);
        double eLen = edge[0].distanceTo(edge[1]);
        for (Vec[] w : walls()) {
            Vec wDir = w[1].sub(w[0]).normalize();
            if (Vec.angleDiff(Vec.lineAngle(eDir), Vec.lineAngle(wDir)) > 2.0) continue;
            Vec wn = new Vec(-wDir.y, wDir.x);
            Vec wMid = w[0].add(w[1]).mul(0.5);
            double dist = Math.abs(eMid.sub(wMid).dot(wn));
            if (dist > 15.0) continue;
            double overlap = Geometry.collinearOverlap(edge[0], edge[1], w[0], w[1], wDir);
            if (overlap >= eLen - 30.0) return true;
        }
        return false;
    }
}
