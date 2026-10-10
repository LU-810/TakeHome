package com.juling.packing;

import java.util.ArrayList;
import java.util.List;

/**
 * 将房间与摆放结果渲染成 SVG 平面图（白底），便于人工核对：
 * 黑色多边形=轮廓，红色线段+圆弧=门，黄色半透明=门禁放区，
 * 蓝色半透明=冰箱开门区，彩色矩形=各物品（标注名称、尺寸、角度）。
 */
public final class SvgWriter {

    private SvgWriter() {}

    public static String write(Room room, List<Placement> result, String title) {
        List<Vec> all = new ArrayList<Vec>();
        for (Vec v : room.boundary) all.add(v);
        all.add(room.doorA);
        all.add(room.doorB);
        for (Obb z : room.blockedZones) {
            for (Vec c : z.corners()) all.add(c);
        }
        if (result != null) {
            for (Placement p : result) {
                for (Vec c : p.body().corners()) all.add(c);
                if (p.fridgeDoorZone != null) {
                    for (Vec c : p.fridgeDoorZone.corners()) all.add(c);
                }
            }
        }
        double[] bb = Geometry.bbox(all);
        double pad = 350;
        double minX = bb[0] - pad, minY = bb[1] - pad;
        double maxX = bb[2] + pad, maxY = bb[3] + pad;
        double w = maxX - minX, h = maxY - minY;

        double targetW = 1500;
        double scale = targetW / w;
        double targetH = h * scale;

        StringBuilder sb = new StringBuilder();
        sb.append(String.format(
                "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" " +
                "viewBox=\"0 0 %.0f %.0f\" font-family=\"Microsoft YaHei, SimHei, Arial, sans-serif\">\n",
                targetW, targetH, targetW, targetH));
        sb.append("<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n");

        // 房间多边形
        StringBuilder poly = new StringBuilder();
        for (int i = 0; i < room.boundary.length; i++) {
            Vec v = room.boundary[i];
            poly.append(i == 0 ? "M" : "L").append(sx(v.x, minX, scale)).append(',')
                .append(sy(v.y, minY, scale, targetH)).append(' ');
        }
        poly.append("Z");
        sb.append("<path d=\"").append(poly).append("\" fill=\"#f7f7f7\" stroke=\"#111\" ")
          .append("stroke-width=\"3\" stroke-linejoin=\"round\"/>\n");

        // 门禁放区
        for (Obb z : room.blockedZones) {
            sb.append(obbPath(z, minX, minY, scale, targetH));
            sb.append(" fill=\"#ffd54f\" fill-opacity=\"0.40\" stroke=\"#e69500\" ")
              .append("stroke-width=\"1.5\" stroke-dasharray=\"8,6\"/>\n");
        }

        // 冰箱开门区
        if (result != null) {
            for (Placement p : result) {
                if (p.fridgeDoorZone == null) continue;
                sb.append(obbPath(p.fridgeDoorZone, minX, minY, scale, targetH));
                sb.append(" fill=\"#82b1ff\" fill-opacity=\"0.30\" stroke=\"#1f6fb2\" ")
                  .append("stroke-width=\"1.5\" stroke-dasharray=\"8,6\"/>\n");
            }
        }

        // 门：红色门洞线 + 门扇弧线
        drawDoor(sb, room, minX, minY, scale, targetH);

        // 物品
        if (result != null) {
            for (Placement p : result) {
                String[] style = styleOf(p.item.type);
                Obb b = p.body();
                sb.append(obbPath(b, minX, minY, scale, targetH));
                sb.append(" fill=\"").append(style[0]).append("\" stroke=\"").append(style[1])
                  .append("\" stroke-width=\"2.5\"/>\n");
                Vec c = b.c;
                double cx = sx(c.x, minX, scale);
                double cy = sy(c.y, minY, scale, targetH);
                sb.append(String.format(
                        "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"middle\" font-size=\"20\" " +
                        "fill=\"#111\" font-weight=\"bold\">%s</text>\n",
                        cx, cy - 6, escape(p.item.name)));
                sb.append(String.format(
                        "<text x=\"%.1f\" y=\"%.1f\" text-anchor=\"middle\" font-size=\"15\" fill=\"#333\">%d×%d  %.1f°</text>\n",
                        cx, cy + 16,
                        Math.round(p.item.length), Math.round(p.item.width), p.angleDeg));
            }
        }

        // 标题与图例
        sb.append("<text x=\"20\" y=\"34\" font-size=\"26\" font-weight=\"bold\" fill=\"#111\">")
          .append(escape(title)).append("</text>\n");
        sb.append("<text x=\"20\" y=\"60\" font-size=\"15\" fill=\"#555\">")
          .append("门宽 N=").append(Math.round(room.doorWidth)).append("mm，")
          .append(room.openInward ? "内开门（禁放区 N×N）" : "外开门（室内保留通行净空）")
          .append("；蓝虚线=冰箱开门区 ").append(Math.round(PackingSolver.FRIDGE_OPEN_DEPTH))
          .append("mm</text>\n");

        sb.append("</svg>\n");
        return sb.toString();
    }

    private static void drawDoor(StringBuilder sb, Room room, double minX, double minY,
                                 double scale, double targetH) {
        Vec a = room.doorA, b = room.doorB;
        sb.append("<line x1=\"").append(sx(a.x, minX, scale)).append("\" y1=\"")
          .append(sy(a.y, minY, scale, targetH)).append("\" x2=\"").append(sx(b.x, minX, scale))
          .append("\" y2=\"").append(sy(b.y, minY, scale, targetH))
          .append("\" stroke=\"#d32f2f\" stroke-width=\"6\" stroke-linecap=\"round\"/>\n");

        // 以 doorA 为铰链，门扇从 doorB 位置扫向室内（内开）或室外（外开）
        Vec side = room.openInward ? room.doorInNormal : room.doorInNormal.neg();
        Vec hinge = a;
        Vec freeClosed = b;
        Vec freeOpen = hinge.add(side.mul(room.doorWidth));
        double x1 = sx(freeClosed.x, minX, scale), y1 = sy(freeClosed.y, minY, scale, targetH);
        double x2 = sx(freeOpen.x, minX, scale), y2 = sy(freeOpen.y, minY, scale, targetH);
        double r = room.doorWidth * scale;
        // sweep 标志：按屏幕坐标（y 向下）判断
        Vec screenDir = new Vec((freeOpen.x - freeClosed.x) * scale,
                                -(freeOpen.y - freeClosed.y) * scale);
        Vec screenD = new Vec((b.x - a.x) * scale, -(b.y - a.y) * scale);
        double sweep = screenD.cross(screenDir) > 0 ? 1 : 0;
        sb.append("<path d=\"M").append(fmt(x1)).append(' ').append(fmt(y1))
          .append(" A").append(fmt(r)).append(' ').append(fmt(r))
          .append(" 0 0 ").append(sweep).append(' ').append(fmt(x2)).append(' ').append(fmt(y2))
          .append("\" fill=\"none\" stroke=\"#d32f2f\" stroke-width=\"1.8\" ")
          .append("stroke-dasharray=\"6,5\"/>\n");
    }

    private static String obbPath(Obb b, double minX, double minY, double scale, double targetH) {
        Vec[] c = b.corners();
        StringBuilder d = new StringBuilder();
        for (int i = 0; i < c.length; i++) {
            d.append(i == 0 ? "M" : "L").append(fmt(sx(c[i].x, minX, scale))).append(' ')
             .append(fmt(sy(c[i].y, minY, scale, targetH))).append(' ');
        }
        d.append("Z");
        return "<path d=\"" + d + "\"";
    }

    private static String[] styleOf(Item.Type t) {
        switch (t) {
            case FRIDGE:     return new String[] { "#cfe8ff", "#1f6fb2" };
            case SHELF:      return new String[] { "#d9f2d9", "#2e7d32" };
            case OVER_SHELF: return new String[] { "#e8dcff", "#6a3fa0" };
            case ICE_MAKER:  return new String[] { "#ffe6cc", "#cc6600" };
            default:         return new String[] { "#eeeeee", "#333333" };
        }
    }

    private static double sx(double x, double minX, double scale) {
        return (x - minX) * scale;
    }

    private static double sy(double y, double minY, double scale, double targetH) {
        return targetH - (y - minY) * scale;
    }

    private static String fmt(double v) {
        return String.format(java.util.Locale.ROOT, "%.1f", v);
    }

    private static String escape(String s) {
        return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
    }
}
