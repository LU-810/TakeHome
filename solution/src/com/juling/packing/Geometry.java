package com.juling.packing;

import java.util.List;

/**
 * 通用二维几何工具：线段相交、点在多边形内、矩形在多边形内判定、室内法线等。
 * 坐标单位按输入约定视为毫米。
 */
public final class Geometry {
    /** 通用点重合 / 边界容差（毫米）。输入数据存在约 0.6mm 的描图噪声，容差不能取得太小 */
    public static final double EPS = 5.0;

    private Geometry() {}

    /** (p,q) 与 (a,b) 的叉积符号：>0 b 在有向线段 p->q 左侧，<0 右侧，=0 共线 */
    public static double orient(Vec p, Vec q, Vec b) {
        return q.sub(p).cross(b.sub(p));
    }

    /** 点 b 是否落在闭线段 p-q 上（含端点），容差 EPS */
    public static boolean onSegment(Vec p, Vec q, Vec b) {
        if (Math.abs(orient(p, q, b)) > EPS) return false;
        return b.distanceToSegment(p, q) <= EPS;
    }

    /**
     * 两条线段是否发生“真正穿越”式相交（proper intersection）。
     * 共线重叠、端点恰好相接都不算穿越——贴墙摆放时物品边与墙共线是合法的。
     */
    public static boolean segmentsCross(Vec p1, Vec q1, Vec p2, Vec q2) {
        double o1 = orient(p1, q1, p2);
        double o2 = orient(p1, q1, q2);
        double o3 = orient(p2, q2, p1);
        double o4 = orient(p2, q2, q1);
        boolean cross1 = (o1 > EPS && o2 < -EPS) || (o1 < -EPS && o2 > EPS);
        boolean cross2 = (o3 > EPS && o4 < -EPS) || (o3 < -EPS && o4 > EPS);
        return cross1 && cross2;
    }

    /**
     * 点是否在多边形内部或边界上（射线法 + 边界容差）。
     * @param poly 按顺序首尾相连的闭合多边形顶点
     */
    public static boolean pointInPolygon(Vec p, Vec[] poly) {
        int n = poly.length;
        // 1) 先判边界
        for (int i = 0; i < n; i++) {
            Vec a = poly[i], b = poly[(i + 1) % n];
            if (p.distanceToSegment(a, b) <= EPS) return true;
        }
        // 2) 射线法（水平向右射线）
        boolean inside = false;
        for (int i = 0; i < n; i++) {
            Vec a = poly[i], b = poly[(i + 1) % n];
            boolean crosses = (a.y > p.y) != (b.y > p.y);
            if (crosses) {
                double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (x > p.x + EPS) inside = !inside;
            }
        }
        return inside;
    }

    /** 有向多边形面积（shoelace），绝对值为面积，符号表示顺逆时针 */
    public static double signedArea(Vec[] poly) {
        double s = 0;
        for (int i = 0; i < poly.length; i++) {
            Vec a = poly[i], b = poly[(i + 1) % poly.length];
            s += a.cross(b);
        }
        return s / 2.0;
    }

    /**
     * 判断定向矩形 rect 是否完全位于多边形 poly 内部（边可与多边形边界重合）。
     * 三层校验，凹多边形也可靠：
     *   1) 四个角点都在多边形内（含边界）；
     *   2) 矩形四条边不与多边形边界发生穿越相交；
     *   3) 矩形内部按固定步长网格采样，所有采样点都在多边形内
     *      （防止凹多边形的凹口被矩形“跨过”而角点仍在内的情形）。
     */
    public static boolean rectInsidePolygon(Obb rect, Vec[] poly, double sampleStep) {
        Vec[] cs = rect.corners();
        for (Vec c : cs) {
            if (!pointInPolygon(c, poly)) return false;
        }
        Vec[][] redges = rect.edges();
        int n = poly.length;
        for (Vec[] re : redges) {
            for (int i = 0; i < n; i++) {
                Vec a = poly[i], b = poly[(i + 1) % n];
                if (segmentsCross(re[0], re[1], a, b)) return false;
            }
        }
        for (Vec sp : rect.samplePoints(sampleStep)) {
            if (!pointInPolygon(sp, poly)) return false;
        }
        return true;
    }

    /**
     * 给定一段墙（有向线段 a->b），返回指向多边形内部的单位法向量。
     * 做法：取两个候选法向，向各自方向偏移一小段，落在多边形内部的即为内法向。
     */
    public static Vec inwardNormal(Vec a, Vec b, Vec[] poly) {
        Vec d = b.sub(a).normalize();
        Vec n1 = new Vec(-d.y, d.x);   // 左侧法向
        Vec mid = a.add(b).mul(0.5);
        double probe = 50.0;
        boolean n1Inside = pointInPolygon(mid.add(n1.mul(probe)), poly);
        boolean n2Inside = pointInPolygon(mid.add(n1.neg().mul(probe)), poly);
        if (n1Inside && !n2Inside) return n1;
        if (!n1Inside && n2Inside) return n1.neg();
        // 两个方向都在内/外（探针落在边界等极端情况）：用多边形绕向兜底
        double area = signedArea(poly);
        // CCW（面积>0，数学坐标 y 向上）内部在有向边左侧
        return area > 0 ? n1 : n1.neg();
    }

    /** 两条共线线段（方向 dir）的投影重叠长度；不共线返回 0 */
    public static double collinearOverlap(Vec a1, Vec a2, Vec b1, Vec b2, Vec dir) {
        double t1 = a1.dot(dir), t2 = a2.dot(dir);
        double t3 = b1.dot(dir), t4 = b2.dot(dir);
        double lo = Math.max(Math.min(t1, t2), Math.min(t3, t4));
        double hi = Math.min(Math.max(t1, t2), Math.max(t3, t4));
        return Math.max(0, hi - lo);
    }

    /** 多边形 bbox：[minX, minY, maxX, maxY] */
    public static double[] bbox(List<Vec> pts) {
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (Vec p : pts) {
            minX = Math.min(minX, p.x);
            minY = Math.min(minY, p.y);
            maxX = Math.max(maxX, p.x);
            maxY = Math.max(maxY, p.y);
        }
        return new double[] { minX, minY, maxX, maxY };
    }
}
