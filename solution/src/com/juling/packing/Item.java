package com.juling.packing;

/**
 * 一件待摆放的物品。
 * type 由物品名称前缀解析：fridge / shelf / overShelf / iceMaker。
 * length、width 单位毫米；angle=0 时 length 边沿 x 轴、width 边沿 y 轴。
 */
public final class Item {
    public enum Type { FRIDGE, SHELF, OVER_SHELF, ICE_MAKER }

    public final String name;
    public final Type type;
    public final double length;
    public final double width;

    public Item(String name, double length, double width) {
        this.name = name;
        this.type = parseType(name);
        this.length = length;
        this.width = width;
    }

    /** 注意必须先判断 overShelf 再判断 shelf，否则 "overShelf-1" 会被误判成货架 */
    private static Type parseType(String name) {
        String n = name.toLowerCase();
        if (n.startsWith("overshelf")) return Type.OVER_SHELF;
        if (n.startsWith("fridge")) return Type.FRIDGE;
        if (n.startsWith("icemaker") || n.startsWith("ice")) return Type.ICE_MAKER;
        if (n.startsWith("shelf")) return Type.SHELF;
        throw new IllegalArgumentException("无法识别的物品类型: " + name);
    }

    public double area() {
        return length * width;
    }

    @Override
    public String toString() {
        return name + "[" + (long) length + "x" + (long) width + "]";
    }
}
