package com.juling.packing;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

/**
 * 通行可达性检查。
 *
 * 做法：在房间包围盒上铺 GRID 栅格，把门摆动区和每件物品本体各外扩
 * PASS_HALF（模拟人通行所需半宽），得到“中心线可通行格”，从门内周边撒
 * 入口种子做四邻域 flood fill。
 *
 * 关键：以“空房间（只有门区）”的可达结果为基准。墙体与门区自身围成的
 * 窄缝（例如斜墙与门端交汇处、人本来就无法利用的建筑死角）在空房间里就
 * 不可达，不追责；只有“空房间能走到、摆上物品后走不到”的格子才算摆放
 * 造成的围堵，判为非法。这样既能抓住冰箱/货架堵门、堵连通口，又不会被
 * 建筑本身的窄缝误报。
 *
 * 冰箱开门区是开门时的临时净空（关门后即普通地面），不作为永久障碍。
 */
public final class Reachability {

    /** 栅格步长，毫米 */
    public static final double GRID = 150.0;
    /** 通行半宽：通道净宽约 2*175 = 350mm（单人侧身可过的下限） */
    public static final double PASS_HALF = 175.0;

    private Reachability() {}

    public static final class Result {
        public boolean ok;
        /** 摆放后灌水可达的格心 */
        public final List<Vec> reachable = new ArrayList<Vec>();
        /** 因摆放而新增的不可达格心（被物品围死的通道/区域） */
        public final List<Vec> dead = new ArrayList<Vec>();
    }

    public static boolean allReachable(Room room, List<Placement> placed) {
        return analyze(room, placed).ok;
    }

    public static Result analyze(Room room, List<Placement> placed) {
        Result res = new Result();
        Vec[] poly = room.boundary;
        double[] bb = bbox(poly);
        double x0 = bb[0], y0 = bb[1], x1 = bb[2], y1 = bb[3];
        int nx = (int) Math.ceil((x1 - x0) / GRID) + 1;
        int ny = (int) Math.ceil((y1 - y0) / GRID) + 1;

        // 仅门区膨胀（空房间基准）
        List<Obb> baseInflated = new ArrayList<Obb>();
        for (Obb z : room.blockedZones) baseInflated.add(inflate(z, PASS_HALF));
        boolean[] pass0 = buildGrid(poly, bb, nx, ny, baseInflated);
        boolean[] seen0 = flood(pass0, nx, ny, entrySeeds(room), x0, y0);

        // 门区 + 已放物品全部膨胀（实际摆放）
        List<Obb> fullInflated = new ArrayList<Obb>(baseInflated);
        for (Placement p : placed) fullInflated.add(inflate(p.body(), PASS_HALF));
        boolean[] pass1 = buildGrid(poly, bb, nx, ny, fullInflated);
        boolean[] seen1 = flood(pass1, nx, ny, entrySeeds(room), x0, y0);

        res.ok = true;
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                int id = j * nx + i;
                Vec c = new Vec(x0 + (i + 0.5) * GRID, y0 + (j + 0.5) * GRID);
                if (pass1[id] && seen1[id]) {
                    res.reachable.add(c);
                } else if (pass0[id] && seen0[id] && pass1[id] && !seen1[id]) {
                    // 空房间走得到、摆放后走不到：摆放围出的死角
                    res.ok = false;
                    res.dead.add(c);
                }
            }
        }
        return res;
    }

    private static boolean[] buildGrid(Vec[] poly, double[] bb, int nx, int ny,
                                       List<Obb> inflated) {
        double x0 = bb[0], y0 = bb[1];
        boolean[] pass = new boolean[nx * ny];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                Vec c = new Vec(x0 + (i + 0.5) * GRID, y0 + (j + 0.5) * GRID);
                int id = j * nx + i;
                if (!Geometry.pointInPolygon(c, poly)) continue;
                boolean bloated = false;
                for (Obb b : inflated) {
                    if (contains(b, c, 2.0)) { bloated = true; break; }
                }
                pass[id] = !bloated;
            }
        }
        return pass;
    }

    private static boolean[] flood(boolean[] pass, int nx, int ny,
                                   List<Vec> seeds, double x0, double y0) {
        boolean[] seen = new boolean[nx * ny];
        Deque<Integer> queue = new ArrayDeque<Integer>();
        for (Vec s : seeds) {
            int i = (int) Math.floor((s.x - x0) / GRID);
            int j = (int) Math.floor((s.y - y0) / GRID);
            if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
            int id = j * nx + i;
            if (pass[id] && !seen[id]) { seen[id] = true; queue.add(id); }
        }
        if (queue.isEmpty()) {
            for (int id = 0; id < pass.length; id++) {
                if (pass[id]) { seen[id] = true; queue.add(id); break; }
            }
        }
        while (!queue.isEmpty()) {
            int id = queue.poll();
            int i = id % nx, j = id / nx;
            int[][] nb = { { i + 1, j }, { i - 1, j }, { i, j + 1 }, { i, j - 1 } };
            for (int[] t : nb) {
                int ni = t[0], nj = t[1];
                if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
                int nid = nj * nx + ni;
                if (pass[nid] && !seen[nid]) { seen[nid] = true; queue.add(nid); }
            }
        }
        return seen;
    }

    /** 在门区室内侧周边撒入口种子（沿法向、沿门轴两个方向外推） */
    private static List<Vec> entrySeeds(Room room) {
        List<Vec> seeds = new ArrayList<Vec>();
        Vec doorMid = room.doorA.add(room.doorB).mul(0.5);
        Vec dd = room.doorB.sub(room.doorA).normalize();
        double doorLen = room.doorA.distanceTo(room.doorB);
        for (Obb z : room.blockedZones) {
            if (z.c.distanceTo(doorMid) > doorLen + 700.0) continue; // 不是门区
            Vec n = z.c.sub(doorMid);
            if (n.lengthSq() < 1e-9) continue;
            n = n.normalize();
            double rd = 0;
            for (Vec cor : z.corners()) {
                rd = Math.max(rd, Math.abs(cor.sub(z.c).dot(dd)));
            }
            for (Vec cor : z.corners()) {
                for (double fn : new double[] { PASS_HALF + 25, PASS_HALF + 175 }) {
                    for (double fd : new double[] { 0.0, rd + PASS_HALF + 25,
                            -(rd + PASS_HALF + 25) }) {
                        seeds.add(cor.add(n.mul(fn)).add(dd.mul(fd)));
                    }
                }
            }
            break; // 题目当前为单门
        }
        return seeds;
    }

    /** 顶点数组的 bbox：[minX, minY, maxX, maxY] */
    private static double[] bbox(Vec[] pts) {
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

    private static Obb inflate(Obb b, double d) {
        return new Obb(b.c, b.u, b.v, b.hl + d, b.hw + d);
    }

    private static boolean contains(Obb b, Vec p, double eps) {
        Vec r = p.sub(b.c);
        return Math.abs(r.dot(b.u)) <= b.hl + eps && Math.abs(r.dot(b.v)) <= b.hw + eps;
    }

    // ------------------------------------------------------------------
    // 独立输出一张通道可达性 SVG（不依赖 SvgWriter）
    // ------------------------------------------------------------------

    public static void writeSvg(String path, Room room, List<Placement> placed) {
        Result res = analyze(room, placed);
        Vec[] poly = room.boundary;
        double[] bb = bbox(poly);
        double x0 = bb[0], y0 = bb[1], x1 = bb[2], y1 = bb[3];
        double w = x1 - x0, h = y1 - y0;
        double maxW = 820, maxH = 1500, pad = 50;
        double scale = Math.min(maxW / w, maxH / h);
        double cw = w * scale + pad * 2;
        double ch = h * scale + pad * 2 + 40;

        StringBuilder sb = new StringBuilder();
        sb.append("<svg xmlns='http://www.w3.org/2000/svg' width='").append((long) cw)
                .append("' height='").append((long) ch)
                .append("' viewBox='0 0 ").append(cw).append(' ').append(ch).append("'>");
        sb.append("<rect width='100%' height='100%' fill='white'/>");

        double gs = GRID * scale;
        for (Vec c : res.reachable) {
            cell(sb, c, x0, y1, scale, pad, gs, "#9ed59e", 0.45);
        }
        for (Vec c : res.dead) {
            cell(sb, c, x0, y1, scale, pad, gs, "#e26d6d", 0.75);
        }

        StringBuilder pts = new StringBuilder();
        for (Vec v : poly) {
            pts.append(sx(v.x, x0, scale, pad)).append(',')
                    .append(sy(v.y, y1, scale, pad)).append(' ');
        }
        sb.append("<polygon points='").append(pts).append("' fill='#f5f5f0' fill-opacity='0.25' stroke='#222' stroke-width='3'/>");

        for (Obb z : room.blockedZones) {
            drawObb(sb, z, x0, y1, scale, pad, "#f2c200", 0.28, "#b89000", "4,4", false);
        }
        for (Placement p : placed) {
            if (p.fridgeDoorZone != null) {
                drawObb(sb, p.fridgeDoorZone, x0, y1, scale, pad, "#5b8fd6", 0.12,
                        "#2b5d9e", "7,5", true);
            }
        }
        for (Placement p : placed) {
            String fill, stroke;
            switch (p.item.type) {
                case FRIDGE:    fill = "#8fb0d6"; stroke = "#274a73"; break;
                case ICE_MAKER: fill = "#9cc9bf"; stroke = "#2e6b5e"; break;
                case OVER_SHELF:fill = "#9ccb86"; stroke = "#3c6e2e"; break;
                default:        fill = "#e6b06a"; stroke = "#8a5a1e"; break;
            }
            drawObb(sb, p.body(), x0, y1, scale, pad, fill, 0.95, stroke, null, false);
            double tx = sx(p.body().c.x, x0, scale, pad);
            double ty = sy(p.body().c.y, y1, scale, pad);
            sb.append("<text x='").append(r(tx)).append("' y='").append(r(ty + 4))
                    .append("' font-size='13' text-anchor='middle' fill='#111' font-family='sans-serif'>")
                    .append(p.item.name).append("</text>");
        }

        sb.append("<text x='").append(pad).append("' y='24' font-size='16' font-family='sans-serif' fill='#111'>")
                .append("通行可达性  （").append(res.ok ? "全部通道可达，人可通行" : "存在被摆放围死的通道（红色）")
                .append("；净宽阈值约350mm，建筑自身窄缝不追责）</text>");
        double lx = cw - 360, ly = 20;
        legend(sb, lx, ly, "#9ed59e", "可通行");
        legend(sb, lx + 90, ly, "#e26d6d", "不可达");
        legend(sb, lx + 170, ly, "#f2c200", "门摆动区");
        legend(sb, lx + 260, ly, "#5b8fd6", "冰箱开门区");
        sb.append("</svg>");

        Writer wri = null;
        try {
            wri = new OutputStreamWriter(new FileOutputStream(path), StandardCharsets.UTF_8);
            wri.write(sb.toString());
        } catch (IOException e) {
            throw new RuntimeException("写 reach.svg 失败: " + path, e);
        } finally {
            if (wri != null) try { wri.close(); } catch (IOException ignore) {}
        }
    }

    private static void cell(StringBuilder sb, Vec c, double x0, double y1,
                             double scale, double pad, double gs, String color, double op) {
        double cx = sx(c.x, x0, scale, pad) - gs / 2;
        double cy = sy(c.y, y1, scale, pad) - gs / 2;
        sb.append("<rect x='").append(r(cx)).append("' y='").append(r(cy))
                .append("' width='").append(r(gs + 0.5)).append("' height='").append(r(gs + 0.5))
                .append("' fill='").append(color).append("' fill-opacity='").append(op).append("'/>");
    }

    private static void legend(StringBuilder sb, double x, double y, String color, String text) {
        sb.append("<rect x='").append(r(x)).append("' y='").append(r(y - 11))
                .append("' width='14' height='14' fill='").append(color).append("' opacity='0.7'/>");
        sb.append("<text x='").append(r(x + 18)).append("' y='").append(r(y))
                .append("' font-size='12' font-family='sans-serif' fill='#222'>").append(text).append("</text>");
    }

    private static void drawObb(StringBuilder sb, Obb b, double x0, double y1,
                                double scale, double pad, String fill, double fillOp,
                                String stroke, String dash, boolean fillNone) {
        StringBuilder pts = new StringBuilder();
        for (Vec cor : b.corners()) {
            pts.append(sx(cor.x, x0, scale, pad)).append(',')
                    .append(sy(cor.y, y1, scale, pad)).append(' ');
        }
        sb.append("<polygon points='").append(pts).append("' fill='")
                .append(fillNone ? "none" : fill).append("' fill-opacity='").append(fillOp)
                .append("' stroke='").append(stroke).append("' stroke-width='2'");
        if (dash != null) sb.append(" stroke-dasharray='").append(dash).append("'");
        sb.append("/>");
    }

    private static double sx(double x, double x0, double scale, double pad) {
        return pad + (x - x0) * scale;
    }

    private static double sy(double y, double y1, double scale, double pad) {
        return pad + (y1 - y) * scale;
    }

    private static String r(double v) {
        return String.format(java.util.Locale.ROOT, "%.1f", v);
    }
}
