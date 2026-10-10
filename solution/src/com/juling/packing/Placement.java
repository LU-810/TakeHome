package com.juling.packing;

/**
 * 一件物品的摆放结果：中心点、旋转角度（度，length 边相对 x 轴的逆时针夹角，归一化到 [0,180)）。
 */
public final class Placement {
    public final Item item;
    public final Vec center;
    public final double angleDeg;
    /** 冰箱专属：开门边前方的禁放区（其他物品不得进入）；非冰箱为 null */
    public final Obb fridgeDoorZone;

    public Placement(Item item, Vec center, double angleDeg, Obb fridgeDoorZone) {
        this.item = item;
        this.center = center;
        this.angleDeg = normalize(angleDeg);
        this.fridgeDoorZone = fridgeDoorZone;
    }

    public Obb body() {
        return Obb.byCenter(center, angleDeg, item.length, item.width);
    }

    public static double normalize(double angleDeg) {
        double a = angleDeg % 180.0;
        if (a < 0) a += 180.0;
        // 去掉浮点误差造成的 179.9999 之类
        if (Math.abs(a - 180.0) < 1e-6) a = 0.0;
        return a;
    }
}
