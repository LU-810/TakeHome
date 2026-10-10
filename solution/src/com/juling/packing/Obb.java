package com.juling.packing;

/**
 * 定向矩形（Oriented Bounding Box）。
 *
 * 局部坐标系：
 *   u 轴 = 物品 length 边方向（angle=0 时与全局 x 轴重合）
 *   v 轴 = 物品 width 边方向（u 逆时针转 90 度）
 * 物品的“旋转角度”即 u 轴相对 x 轴的逆时针夹角。
 */
public final class Obb {
    public Vec c;          // 中心点
    public Vec u;          // length 方向单位向量
    public Vec v;          // width 方向单位向量
    public final double hl; // 沿 u 的半长 = length/2
    public final double hw; // 沿 v 的半宽 = width/2

    public Obb(Vec c, Vec u, Vec v, double hl, double hw) {
        this.c = c;
        this.u = u;
        this.v = v;
        this.hl = hl;
        this.hw = hw;
    }

    /** 按中心点、旋转角（度）、长和宽构造 */
    public static Obb byCenter(Vec center, double angleDeg, double length, double width) {
        double rad = Math.toRadians(angleDeg);
        Vec u = new Vec(Math.cos(rad), Math.sin(rad));
        Vec v = new Vec(-Math.sin(rad), Math.cos(rad));
        return new Obb(center, u, v, length / 2.0, width / 2.0);
    }

    /** 四个角点，顺序：(+u,+v) (-u,+v) (-u,-v) (+u,-v) */
    public Vec[] corners() {
        Vec au = u.mul(hl), av = v.mul(hw);
        return new Vec[] {
            c.add(au).add(av),
            c.add(au.neg()).add(av),
            c.add(au.neg()).add(av.neg()),
            c.add(au).add(av.neg())
        };
    }

    /** 四条边（闭合） */
    public Vec[][] edges() {
        Vec[] p = corners();
        return new Vec[][] {
            {p[0], p[1]}, {p[1], p[2]}, {p[2], p[3]}, {p[3], p[0]}
        };
    }

    /** 局部坐标：全局点 -> (沿 u 的距离, 沿 v 的距离) */
    public Vec toLocal(Vec p) {
        Vec d = p.sub(c);
        return new Vec(d.dot(u), d.dot(v));
    }

    public boolean containsPoint(Vec p) {
        Vec l = toLocal(p);
        return Math.abs(l.x) <= hl + Geometry.EPS && Math.abs(l.y) <= hw + Geometry.EPS;
    }

    /**
     * 两个定向矩形是否重叠（SAT 分离轴定理）。
     * @param touchTol 接触容差：两矩形间隙 >= -touchTol（即可轻微嵌入 touchTol 以内）都视为“刚好贴紧、不碰撞”。
     *                 传 0 表示严格接触即算碰撞；传正数允许贴紧。
     * @return true 表示发生实质性重叠
     */
    public boolean intersects(Obb o, double touchTol) {
        Vec[] axes = { u, v, o.u, o.v };
        for (Vec axis : axes) {
            double[] a = project(axis);
            double[] b = o.project(axis);
            double gap = Math.max(a[0], b[0]) - Math.min(a[1], b[1]);
            // gap>0 两区间分离；这里取负号：gap <= -touchTol 才算重叠
            if (gap > -touchTol) {
                return false; // 找到分离轴 => 不相交
            }
        }
        return true;
    }

    /** 矩形在 axis 上的投影区间 [min,max] */
    public double[] project(Vec axis) {
        Vec[] p = corners();
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (Vec q : p) {
            double t = q.dot(axis);
            min = Math.min(min, t);
            max = Math.max(max, t);
        }
        return new double[] { min, max };
    }

    /** 沿 u 方向（length 方向）内部采样点，步长 step；用于“矩形是否完全在房间内”的光栅校验 */
    public java.util.List<Vec> samplePoints(double step) {
        java.util.List<Vec> pts = new java.util.ArrayList<Vec>();
        int nx = Math.max(1, (int) Math.ceil((hl * 2) / step));
        int ny = Math.max(1, (int) Math.ceil((hw * 2) / step));
        for (int i = 0; i <= nx; i++) {
            double lx = -hl + (hl * 2.0) * i / nx;
            for (int j = 0; j <= ny; j++) {
                double ly = -hw + (hw * 2.0) * j / ny;
                pts.add(c.add(u.mul(lx)).add(v.mul(ly)));
            }
        }
        return pts;
    }
}
