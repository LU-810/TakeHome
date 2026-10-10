package com.juling.packing;

/**
 * 二维向量 / 点。全部工具方法均不修改原对象，返回新对象。
 */
public final class Vec {
    public final double x;
    public final double y;

    public Vec(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec add(Vec o) { return new Vec(x + o.x, y + o.y); }

    public Vec sub(Vec o) { return new Vec(x - o.x, y - o.y); }

    public Vec mul(double k) { return new Vec(x * k, y * k); }

    public Vec neg() { return new Vec(-x, -y); }

    public double dot(Vec o) { return x * o.x + y * o.y; }

    /** 二维叉积（z 分量）：>0 表示 o 在 this 的逆时针方向 */
    public double cross(Vec o) { return x * o.y - y * o.x; }

    public double length() { return Math.hypot(x, y); }

    public double lengthSq() { return x * x + y * y; }

    public Vec normalize() {
        double len = length();
        return len < 1e-12 ? new Vec(0, 0) : new Vec(x / len, y / len);
    }

    /** 逆时针旋转 rad 弧度 */
    public Vec rotate(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return new Vec(x * c - y * s, x * s + y * c);
    }

    /** 向量方向角（度），归一化到 [0,180)：直线没有方向，180 度等价于 0 度 */
    public static double lineAngle(Vec d) {
        double a = Math.toDegrees(Math.atan2(d.y, d.x));
        a %= 180.0;
        if (a < 0) a += 180.0;
        return a;
    }

    /** 方向角差（度），结果落在 [0,90]：两条直线的最小夹角 */
    public static double angleDiff(double a, double b) {
        double d = Math.abs(a - b) % 180.0;
        if (d > 90.0) d = 180.0 - d;
        return d;
    }

    public double distanceTo(Vec o) { return Math.hypot(x - o.x, y - o.y); }

    /** 点到线段 p-q 的最短距离 */
    public double distanceToSegment(Vec p, Vec q) {
        Vec d = q.sub(p);
        double l2 = d.lengthSq();
        if (l2 < 1e-12) return distanceTo(p);
        double t = clamp(this.sub(p).dot(d) / l2, 0.0, 1.0);
        return distanceTo(p.add(d.mul(t)));
    }

    public static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}
