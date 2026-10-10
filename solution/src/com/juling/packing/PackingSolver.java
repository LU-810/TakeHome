package com.juling.packing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * 摆放求解器 V3。
 *
 * 阶段一：大件（冰箱、制冰机）先放。候选由「贴墙滑动 + 角点对齐（含门区
 *   角点）」枚举，评分包含“放完后各墙面剩余可成列长度”，迫使大件占端部/
 *   角部、把最长连续墙让给货架；并强制：冰箱正面 800mm 开门区在室内、不压
 *   门摆动区、不压其他物品；摆完后全房间通行可达（Reachability）。
 * 阶段二：条带（货架、离地架）墙面成列规划。探针扫描可用连续区间，货架按
 *   长度降序 best-fit 成列（远离基准角端起排），离地架填基准角碎区；每放一
 *   件都重新做通行可达性校验，堵门、堵连通口的摆法直接丢弃。
 * 大件摆法回溯；阶段二失败则退回逐件候选 + 接续候选的贪心回溯兜底。
 */
public final class PackingSolver {

    public static final double FRIDGE_OPEN_DEPTH = 800.0;
    public static final double TOUCH_TOL = 2.0;
    private static final double SCAN_STEP = 50.0;
    private static final int FALLBACK_TOP_N = 30;
    private static final int FALLBACK_BUDGET = 120000;

    private final Room room;
    private final List<Item> items;
    private final List<Placement> placed = new ArrayList<Placement>();
    private final List<Item> largeItems = new ArrayList<Item>();
    private final List<Item> stripItems = new ArrayList<Item>();
    private final List<Item> fallbackOrder = new ArrayList<Item>();
    private final Vec anchor;
    private int nodesUsed;

    public PackingSolver(Room room, List<Item> items) {
        this.room = room;
        this.items = new ArrayList<Item>(items);
        for (Item it : this.items) {
            if (it.type == Item.Type.FRIDGE || it.type == Item.Type.ICE_MAKER) {
                largeItems.add(it);
            } else {
                stripItems.add(it);
            }
        }
        Collections.sort(largeItems, new Comparator<Item>() {
            public int compare(Item a, Item b) {
                int c = Double.compare(b.area(), a.area());
                return c != 0 ? c : a.name.compareTo(b.name);
            }
        });
        Collections.sort(stripItems, new Comparator<Item>() {
            public int compare(Item a, Item b) {
                int c = Double.compare(b.length, a.length);
                if (c != 0) return c;
                c = Double.compare(b.width, a.width);
                return c != 0 ? c : a.name.compareTo(b.name);
            }
        });
        List<Item> stripsAsc = new ArrayList<Item>(stripItems);
        Collections.sort(stripsAsc, new Comparator<Item>() {
            public int compare(Item a, Item b) {
                int c = Double.compare(a.length, b.length);
                if (c != 0) return c;
                return a.name.compareTo(b.name);
            }
        });
        fallbackOrder.addAll(largeItems);
        fallbackOrder.addAll(stripsAsc);

        List<Vec> poly = new ArrayList<Vec>();
        for (Vec v : room.boundary) poly.add(v);
        double[] bb = Geometry.bbox(poly);
        this.anchor = new Vec(bb[0], bb[1]);
    }

    public List<Item> orderedItems() {
        List<Item> r = new ArrayList<Item>();
        r.addAll(largeItems);
        r.addAll(stripItems);
        return r;
    }

    public List<Placement> solve() {
        nodesUsed = 0;
        placed.clear();
        if (placeLarge(0)) return new ArrayList<Placement>(placed);
        placed.clear();
        nodesUsed = 0;
        if (fallbackDfs(0)) return new ArrayList<Placement>(placed);
        return null;
    }

    // ---------------- 阶段一 ----------------

    private boolean placeLarge(int idx) {
        if (++nodesUsed > FALLBACK_BUDGET) return false;
        if (idx == largeItems.size()) {
            int snapshot = placed.size();
            if (planStrips()) return true;
            while (placed.size() > snapshot) placed.remove(placed.size() - 1);
            return false;
        }
        Item item = largeItems.get(idx);
        List<Candidate> cands = generateLargeCandidates(item);
        sortCandidates(cands);
        for (Candidate cand : cands) {
            Placement p = commit(cand);
            if (p == null) continue;
            placed.add(p);
            if (!Reachability.allReachable(room, placed)) {
                placed.remove(placed.size() - 1);
                continue;
            }
            if (placeLarge(idx + 1)) return true;
            placed.remove(placed.size() - 1);
        }
        return false;
    }

    private List<Candidate> generateLargeCandidates(Item item) {
        List<Candidate> out = new ArrayList<Candidate>();
        Set<String> dedup = new LinkedHashSet<String>();
        for (Vec[] w : room.walls()) {
            Vec a = w[0], b = w[1];
            Vec d = b.sub(a).normalize();
            if (d.lengthSq() < 1e-12) continue;
            Vec n = Geometry.inwardNormal(a, b, room.boundary);
            double len = a.distanceTo(b);
            double[] feats = featureParams(a, d);
            addSliding(out, dedup, item, Vec.lineAngle(b.sub(a)),
                    a, d, n, len, item.length, item.width / 2.0, feats, true);
            double perp = Placement.normalize(Vec.lineAngle(b.sub(a)) - 90.0);
            addSliding(out, dedup, item, perp,
                    a, d, n, len, item.width, item.length / 2.0, feats, true);
        }
        List<Vec> seeds = new ArrayList<Vec>();
        Collections.addAll(seeds, room.boundary);
        for (Obb z : room.blockedZones) Collections.addAll(seeds, z.corners());
        for (Placement p : placed) Collections.addAll(seeds, p.body().corners());
        List<Double> angles = room.wallAngles();
        int na = angles.size();
        for (int k = 0; k < na; k++) angles.add(Placement.normalize(angles.get(k) - 90.0));
        for (double angle : angles) {
            Vec[] off = Obb.byCenter(new Vec(0, 0), angle, item.length, item.width).corners();
            for (Vec seed : seeds)
                for (Vec o : off)
                    considerLarge(out, dedup, item, seed.sub(o), angle);
        }
        return out;
    }

    private void considerLarge(List<Candidate> out, Set<String> dedup, Item item,
                               Vec center, double angle) {
        angle = Placement.normalize(angle);
        String key = Math.round(center.x) + "_" + Math.round(center.y) + "_" + Math.round(angle * 10);
        if (!dedup.add(key)) return;
        Obb body = Obb.byCenter(center, angle, item.length, item.width);
        if (!bodyValid(body)) return;

        Obb fridgeZone = null;
        double penalty = 0;
        if (item.type == Item.Type.FRIDGE) {
            Obb[] z = buildFridgeDoorZone(body);
            if (z == null) return;
            fridgeZone = z[0];
            // 开门区不得压门摆动区（两门不能同时打架）
            for (Obb bz : room.blockedZones) {
                if (fridgeZone.intersects(bz, TOUCH_TOL)) return;
            }
            for (Placement q : placed) {
                if (fridgeZone.intersects(q.body(), TOUCH_TOL)) return;
            }
            if (z[1] == null) penalty = 5000.0;
        }
        Placement tmp = new Placement(item, center, angle, fridgeZone);
        placed.add(tmp);
        double wallContact = room.wallContactLength(body);
        double corner = cornerBonus(body);
        int packed = packStripsCount();
        int missing = stripItems.size() - packed;
        boolean reach = Reachability.allReachable(room, placed);
        placed.remove(placed.size() - 1);
        if (!reach) return;

        // 条带每能成箱放下一件给 6000；只要有一件放不下就重罚 40000，
        // 迫使大件把连续长墙让给条带组（压过单纯占角的 8000 偏好）
        double score = wallContact * 10.0 + corner + packed * 6000.0
                - body.c.distanceTo(anchor) * 0.05 - penalty - missing * 40000.0;
        out.add(new Candidate(item, center, angle, fridgeZone, score));
    }

    private double cornerBonus(Obb body) {
        Vec[][] e = body.edges();
        boolean[] ag = new boolean[4];
        for (int i = 0; i < 4; i++) ag[i] = room.edgeAgainstWall(e[i]);
        for (int i = 0; i < 4; i++)
            if (ag[i] && ag[(i + 1) % 4]) return 8000.0;
        return 0.0;
    }

    // ---------------- 阶段二 ----------------

    private List<double[]> scanWall(Vec[] w, double depth) {
        Vec a = w[0], b = w[1];
        Vec d = b.sub(a).normalize();
        if (d.lengthSq() < 1e-12) return Collections.emptyList();
        Vec n = Geometry.inwardNormal(a, b, room.boundary);
        double len = a.distanceTo(b);
        List<Double> ok = new ArrayList<Double>();
        for (double s = SCAN_STEP / 2.0; s <= len - SCAN_STEP / 2.0 + 1e-6; s += SCAN_STEP) {
            Obb probe = new Obb(a.add(d.mul(s)).add(n.mul(depth / 2.0)), d, n,
                    SCAN_STEP / 2.0, depth / 2.0);
            if (probeValid(probe)) ok.add(s);
        }
        List<double[]> ivs = new ArrayList<double[]>();
        if (ok.isEmpty()) return ivs;
        double start = ok.get(0), prev = ok.get(0);
        for (int i = 1; i < ok.size(); i++) {
            double s = ok.get(i);
            if (s - prev > SCAN_STEP * 1.6) {
                ivs.add(new double[] { Math.max(0, start - SCAN_STEP / 2.0),
                        Math.min(len, prev + SCAN_STEP / 2.0) });
                start = s;
            }
            prev = s;
        }
        ivs.add(new double[] { Math.max(0, start - SCAN_STEP / 2.0),
                Math.min(len, prev + SCAN_STEP / 2.0) });
        return ivs;
    }

    /**
     * 条带按类型成组规划：同一类型（货架组 / 离地架组）优先在同一面墙的一段
     * 连续区间里首尾相接、整组成列；整组一次放不下再退到次优墙。每放一件都
     * 做通行可达性校验。这样长条会自然集中到最长的连续墙、短条填其余墙，
     * 与 CAD 例图“沿长墙成列”的做法一致。
     */
    private boolean planStrips() {
        List<List<Item>> groups = new ArrayList<List<Item>>();
        for (Item it : stripItems) {
            List<Item> g = null;
            if (!groups.isEmpty() && groups.get(groups.size() - 1).get(0).type == it.type) {
                g = groups.get(groups.size() - 1);
            }
            if (g == null) {
                g = new ArrayList<Item>();
                groups.add(g);
            }
            g.add(it);
        }

        for (List<Item> group : groups) {
            Item proto = group.get(0);
            int remaining = group.size();
            int guard = 0;
            while (remaining > 0 && guard++ < 30) {
                List<Plan> plans = new ArrayList<Plan>();
                int baseIdx = group.size() - remaining;

                for (Vec[] w : room.walls()) {
                    Vec a = w[0], b = w[1];
                    Vec d = b.sub(a).normalize();
                    if (d.lengthSq() < 1e-12) continue;
                    Vec n = Geometry.inwardNormal(a, b, room.boundary);
                    double angle = Vec.lineAngle(b.sub(a));
                    double wallLen = a.distanceTo(b);
                    double anchorS = anchor.sub(a).dot(d);
                    double farSign = proto.type == Item.Type.OVER_SHELF ? -1.0 : 1.0;
                    double L = proto.length, dep = proto.width;

                    for (double[] iv : scanWall(w, dep)) {
                        int cap = (int) Math.floor((iv[1] - iv[0]) / L + 1e-9);
                        if (cap <= 0) continue;
                        for (int k = cap; k >= 1; k--) {
                            int kk = Math.min(k, remaining);
                            for (int end = 0; end < 2; end++) {
                                List<Placement> poses = new ArrayList<Placement>();
                                boolean valid = true;
                                for (int j = 0; j < kk; j++) {
                                    double s = end == 0
                                            ? iv[0] + L / 2.0 + j * L
                                            : iv[1] - L / 2.0 - j * L;
                                    Vec center = a.add(d.mul(s)).add(n.mul(dep / 2.0));
                                    Obb body = Obb.byCenter(center, angle, proto.length, proto.width);
                                    if (!bodyValid(body)) { valid = false; break; }
                                    poses.add(new Placement(group.get(baseIdx + j), center, angle, null));
                                }
                                if (!valid || poses.isEmpty()) continue;

                                // firstS 始终是序列首件（j=0）沿墙位置，与 firstBody 对应
                                double firstS = end == 0 ? iv[0] + L / 2.0 : iv[1] - L / 2.0;
                                Obb firstBody = poses.get(0).body();
                                double sc = kk * 10000.0;
                                if (kk == remaining) sc += 20000.0;            // 整组一次成列
                                sc += wallLen * 0.05;                          // 优先占更长的主墙成列
                                if (touchesSameType(firstBody, proto, d)) sc += 6000.0;
                                if (sameWallHasStrip(firstBody, d)) sc += 1000.0;
                                sc += farSign * Math.abs(firstS - anchorS) * 0.1;
                                sc -= (iv[1] - iv[0] - kk * L) * 0.05;
                                sc -= firstBody.c.distanceTo(anchor) * 0.02;
                                plans.add(new Plan(poses, sc));
                            }
                        }
                    }
                }

                Collections.sort(plans, new Comparator<Plan>() {
                    public int compare(Plan a, Plan b) { return Double.compare(b.score, a.score); }
                });

                boolean progressed = false;
                for (Plan pl : plans) {
                    List<Placement> added = new ArrayList<Placement>();
                    boolean ok = true;
                    for (Placement p : pl.poses) {
                        Placement c = commitPlacement(p);
                        if (c == null) { ok = false; break; }
                        placed.add(c);
                        added.add(c);
                        if (!Reachability.allReachable(room, placed)) { ok = false; break; }
                    }
                    if (ok) {
                        remaining -= added.size();
                        progressed = true;
                        break;
                    }
                    for (int q = 0; q < added.size(); q++) placed.remove(placed.size() - 1);
                }
                if (!progressed) return false;
            }
            if (remaining > 0) return false;
        }
        return true;
    }

    /**
     * 大件评分用：把所有条带按长度降序、best-fit 装箱到当前各墙 400 进深的
     * 可用连续区间里，返回能放下的条带件数。用于判断大件是否占掉了条带组
     * 成列所必需的连续墙。
     */
    private int packStripsCount() {
        List<Double> segs = new ArrayList<Double>();
        for (Vec[] w : room.walls())
            for (double[] iv : scanWall(w, 400.0)) segs.add(iv[1] - iv[0]);
        List<Double> lengths = new ArrayList<Double>();
        for (Item it : stripItems) lengths.add(it.length);
        Collections.sort(lengths, Collections.reverseOrder());
        int placedCount = 0;
        for (double L : lengths) {
            int best = -1;
            double bestRem = Double.POSITIVE_INFINITY;
            for (int i = 0; i < segs.size(); i++) {
                double rem = segs.get(i);
                if (rem >= L - Geometry.EPS && rem < bestRem) { best = i; bestRem = rem; }
            }
            if (best >= 0) {
                segs.set(best, segs.get(best) - L);
                placedCount++;
            }
        }
        return placedCount;
    }

    private boolean touchesSameType(Obb body, Item item, Vec d) {
        double[] mine = projection(body, d);
        for (Placement p : placed) {
            if (p.item.type != item.type) continue;
            double[] th = projection(p.body(), d);
            boolean adjacent = Math.abs(mine[0] - th[1]) <= 30.0
                    || Math.abs(mine[1] - th[0]) <= 30.0;
            if (adjacent) {
                Vec n = new Vec(-d.y, d.x);
                if (Math.abs(body.c.sub(p.body().c).dot(n)) <= 30.0) return true;
            }
        }
        return false;
    }

    private boolean sameWallHasStrip(Obb body, Vec d) {
        for (Placement p : placed) {
            if (p.item.type == Item.Type.FRIDGE || p.item.type == Item.Type.ICE_MAKER) continue;
            boolean parallel = Math.abs(p.body().u.dot(d)) > 0.99
                    || Math.abs(p.body().v.dot(d)) > 0.99;
            if (parallel) {
                Vec n = new Vec(-d.y, d.x);
                if (Math.abs(body.c.sub(p.body().c).dot(n)) <= 60.0) return true;
            }
        }
        return false;
    }

    private double[] projection(Obb body, Vec d) {
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (Vec c : body.corners()) {
            double t = c.dot(d);
            lo = Math.min(lo, t);
            hi = Math.max(hi, t);
        }
        return new double[] { lo, hi };
    }

    // ---------------- 兜底 ----------------

    private boolean fallbackDfs(int idx) {
        if (++nodesUsed > FALLBACK_BUDGET) return false;
        if (idx == fallbackOrder.size()) return true;
        Item item = fallbackOrder.get(idx);
        List<Candidate> cands = generateFallbackCandidates(item);
        sortCandidates(cands);
        int limit = Math.min(FALLBACK_TOP_N, cands.size());
        for (int i = 0; i < limit; i++) {
            Placement p = commit(cands.get(i));
            if (p == null) continue;
            placed.add(p);
            if (!Reachability.allReachable(room, placed)) {
                placed.remove(placed.size() - 1);
                continue;
            }
            if (fallbackDfs(idx + 1)) return true;
            placed.remove(placed.size() - 1);
        }
        return false;
    }

    private List<Candidate> generateFallbackCandidates(Item item) {
        List<Candidate> out = new ArrayList<Candidate>();
        Set<String> dedup = new LinkedHashSet<String>();
        for (Vec[] w : room.walls()) {
            Vec a = w[0], b = w[1];
            Vec d = b.sub(a).normalize();
            if (d.lengthSq() < 1e-12) continue;
            Vec n = Geometry.inwardNormal(a, b, room.boundary);
            double len = a.distanceTo(b);
            double[] feats = featureParams(a, d);
            addSliding(out, dedup, item, Vec.lineAngle(b.sub(a)),
                    a, d, n, len, item.length, item.width / 2.0, feats, false);
            // 条带只允许长边贴墙；大件（冰箱/制冰机）额外允许垂直朝向
            if (item.type == Item.Type.FRIDGE || item.type == Item.Type.ICE_MAKER) {
                double perp = Placement.normalize(Vec.lineAngle(b.sub(a)) - 90.0);
                addSliding(out, dedup, item, perp,
                        a, d, n, len, item.width, item.length / 2.0, feats, false);
            }
        }
        // 接续候选：与已放条带在同一墙面首尾相接
        for (Placement p : placed) {
            if (p.item.type == Item.Type.FRIDGE) continue;
            for (Vec[] edge : p.body().edges()) {
                Vec[] wall = matchWall(edge);
                if (wall == null) continue;
                Vec a = wall[0], b = wall[1];
                Vec d = b.sub(a).normalize();
                Vec n = Geometry.inwardNormal(a, b, room.boundary);
                double angle = Vec.lineAngle(b.sub(a));
                double along = item.length, off = item.width / 2.0;
                double hp = p.body().hl * Math.abs(p.body().u.dot(d))
                        + p.body().hw * Math.abs(p.body().v.dot(d));
                double centerS = p.body().c.sub(a).dot(d);
                for (int side = -1; side <= 1; side += 2) {
                    double s = centerS + side * (hp + along / 2.0);
                    Vec center = a.add(d.mul(s)).add(n.mul(off));
                    considerFallback(out, dedup, item, center, angle, true);
                }
            }
        }
        List<Vec> seeds = new ArrayList<Vec>();
        Collections.addAll(seeds, room.boundary);
        for (Obb z : room.blockedZones) Collections.addAll(seeds, z.corners());
        for (Placement p : placed) Collections.addAll(seeds, p.body().corners());
        List<Double> angles = room.wallAngles();
        int na = angles.size();
        for (int k = 0; k < na; k++) angles.add(Placement.normalize(angles.get(k) - 90.0));
        for (double angle : angles) {
            Vec[] off = Obb.byCenter(new Vec(0, 0), angle, item.length, item.width).corners();
            for (Vec seed : seeds)
                for (Vec o : off)
                    considerFallback(out, dedup, item, seed.sub(o), angle, false);
        }
        Collections.sort(out, new Comparator<Candidate>() {
            public int compare(Candidate a, Candidate b) {
                return Boolean.compare(b.continuation, a.continuation);
            }
        });
        return out;
    }

    private void considerFallback(List<Candidate> out, Set<String> dedup, Item item,
                                  Vec center, double angle, boolean continuation) {
        angle = Placement.normalize(angle);
        String key = Math.round(center.x) + "_" + Math.round(center.y) + "_" + Math.round(angle * 10);
        if (!dedup.add(key)) return;
        Obb body = Obb.byCenter(center, angle, item.length, item.width);
        if (!bodyValid(body)) return;
        Obb zone = null;
        double penalty = 0;
        if (item.type == Item.Type.FRIDGE) {
            Obb[] z = buildFridgeDoorZone(body);
            if (z == null) return;
            zone = z[0];
            for (Obb bz : room.blockedZones) {
                if (zone.intersects(bz, TOUCH_TOL)) return;
            }
            for (Placement q : placed) {
                if (zone.intersects(q.body(), TOUCH_TOL)) return;
            }
            if (z[1] == null) penalty = 5000.0;
        }
        double wallContact = room.wallContactLength(body);
        double itemContact = 0;
        for (Placement p : placed) itemContact += contactLength(body, p.body());
        double score = wallContact * 10.0 + itemContact * 3.0
                - body.c.distanceTo(anchor) * 0.05 - penalty
                + (continuation ? 25000.0 : 0.0);
        out.add(new Candidate(item, center, angle, zone, score, continuation));
    }

    private Vec[] matchWall(Vec[] edge) {
        Vec eDir = edge[1].sub(edge[0]).normalize();
        Vec eMid = edge[0].add(edge[1]).mul(0.5);
        Vec[] best = null;
        double bestOverlap = 0;
        for (Vec[] w : room.walls()) {
            Vec wDir = w[1].sub(w[0]).normalize();
            if (Vec.angleDiff(Vec.lineAngle(eDir), Vec.lineAngle(wDir)) > 2.0) continue;
            Vec wn = new Vec(-wDir.y, wDir.x);
            Vec wMid = w[0].add(w[1]).mul(0.5);
            if (Math.abs(eMid.sub(wMid).dot(wn)) > 15.0) continue;
            double ov = Geometry.collinearOverlap(edge[0], edge[1], w[0], w[1], wDir);
            if (ov > bestOverlap) {
                bestOverlap = ov;
                best = w;
            }
        }
        return best;
    }

    // ---------------- 公共 ----------------

    private void addSliding(List<Candidate> out, Set<String> dedup, Item item, double angle,
                            Vec a, Vec d, Vec n, double wallLen,
                            double along, double off, double[] features, boolean large) {
        List<Double> ss = new ArrayList<Double>();
        ss.add(along / 2.0);
        ss.add(wallLen - along / 2.0);
        for (double f : features) {
            ss.add(f - along / 2.0);
            ss.add(f + along / 2.0);
        }
        for (double s : ss) {
            if (s - along / 2.0 < -10.0 || s + along / 2.0 > wallLen + 10.0) continue;
            Vec center = a.add(d.mul(s)).add(n.mul(off));
            if (large) considerLarge(out, dedup, item, center, angle);
            else considerFallback(out, dedup, item, center, angle, false);
        }
    }

    private double[] featureParams(Vec a, Vec d) {
        List<Vec> fs = new ArrayList<Vec>();
        Collections.addAll(fs, room.boundary);
        fs.add(room.doorA);
        fs.add(room.doorB);
        for (Obb z : room.blockedZones) Collections.addAll(fs, z.corners());
        for (Placement p : placed) Collections.addAll(fs, p.body().corners());
        double[] r = new double[fs.size()];
        for (int k = 0; k < fs.size(); k++) r[k] = fs.get(k).sub(a).dot(d);
        return r;
    }

    private boolean bodyValid(Obb body) {
        if (!Geometry.rectInsidePolygon(body, room.boundary, Room.SAMPLE_STEP)) return false;
        for (Obb zone : room.blockedZones)
            if (body.intersects(zone, Geometry.EPS)) return false;
        for (Placement p : placed) {
            if (body.intersects(p.body(), TOUCH_TOL)) return false;
            if (p.fridgeDoorZone != null && body.intersects(p.fridgeDoorZone, TOUCH_TOL))
                return false;
        }
        return true;
    }

    private boolean probeValid(Obb probe) {
        if (!Geometry.rectInsidePolygon(probe, room.boundary, 200.0)) return false;
        for (Obb zone : room.blockedZones)
            if (probe.intersects(zone, Geometry.EPS)) return false;
        for (Placement p : placed) {
            if (probe.intersects(p.body(), TOUCH_TOL)) return false;
            if (p.fridgeDoorZone != null && probe.intersects(p.fridgeDoorZone, TOUCH_TOL))
                return false;
        }
        return true;
    }

    private Placement commit(Candidate cand) {
        Obb body = Obb.byCenter(cand.center, cand.angle, cand.item.length, cand.item.width);
        if (!bodyValid(body)) return null;
        if (cand.fridgeZone != null) {
            for (Obb bz : room.blockedZones)
                if (cand.fridgeZone.intersects(bz, TOUCH_TOL)) return null;
            for (Placement p : placed)
                if (cand.fridgeZone.intersects(p.body(), TOUCH_TOL)) return null;
        }
        return new Placement(cand.item, cand.center, cand.angle, cand.fridgeZone);
    }

    private Placement commitPlacement(Placement p) {
        if (!bodyValid(p.body())) return null;
        if (p.fridgeDoorZone != null) {
            for (Obb bz : room.blockedZones)
                if (p.fridgeDoorZone.intersects(bz, TOUCH_TOL)) return null;
            for (Placement q : placed)
                if (p.fridgeDoorZone.intersects(q.body(), TOUCH_TOL)) return null;
        }
        return p;
    }

    // ---------------- 冰箱开门区 ----------------

    /** @return [开门区 Obb, 背面贴墙标记（非 null 表示背面依托墙）]；无法开门返回 null */
    private Obb[] buildFridgeDoorZone(Obb body) {
        Vec[][] edges = body.edges();
        Vec[] plusV = edges[0];
        Vec[] minusV = edges[2];
        boolean plusWall = room.edgeAgainstWall(plusV);
        boolean minusWall = room.edgeAgainstWall(minusV);
        if (plusWall && minusWall) return null;

        Vec[] doorEdge;
        Vec openDir;
        boolean backToWall;
        if (plusWall) {
            doorEdge = minusV; openDir = body.v.neg(); backToWall = true;
        } else if (minusWall) {
            doorEdge = plusV; openDir = body.v; backToWall = true;
        } else {
            if (minWallDistance(mid(plusV)) >= minWallDistance(mid(minusV))) {
                doorEdge = plusV; openDir = body.v;
            } else {
                doorEdge = minusV; openDir = body.v.neg();
            }
            backToWall = false;
        }
        Vec edgeMid = mid(doorEdge);
        for (int k = 1; k <= 8; k++) {
            Vec t = edgeMid.add(openDir.mul(FRIDGE_OPEN_DEPTH * k / 8.0));
            if (!Geometry.pointInPolygon(t, room.boundary)) return null;
        }
        Vec zoneCenter = body.c.add(openDir.mul(body.hw + FRIDGE_OPEN_DEPTH / 2.0));
        Obb zone = new Obb(zoneCenter, body.u, openDir, body.hl, FRIDGE_OPEN_DEPTH / 2.0);
        return new Obb[] { zone, backToWall ? zone : null };
    }

    private Vec mid(Vec[] edge) {
        return edge[0].add(edge[1]).mul(0.5);
    }

    private double minWallDistance(Vec p) {
        double best = Double.POSITIVE_INFINITY;
        for (Vec[] w : room.walls())
            best = Math.min(best, p.distanceToSegment(w[0], w[1]));
        return best;
    }

    private double contactLength(Obb a, Obb b) {
        double total = 0;
        for (Vec[] ea : a.edges()) {
            Vec da = ea[1].sub(ea[0]).normalize();
            Vec ma = mid(ea);
            double best = 0;
            for (Vec[] eb : b.edges()) {
                Vec db = eb[1].sub(eb[0]).normalize();
                if (Vec.angleDiff(Vec.lineAngle(da), Vec.lineAngle(db)) > 2.0) continue;
                Vec normal = new Vec(-da.y, da.x);
                if (Math.abs(mid(eb).sub(ma).dot(normal)) > 25.0) continue;
                best = Math.max(best, Geometry.collinearOverlap(ea[0], ea[1], eb[0], eb[1], da));
            }
            total += best;
        }
        return total;
    }

    private void sortCandidates(List<Candidate> list) {
        Collections.sort(list, new Comparator<Candidate>() {
            public int compare(Candidate a, Candidate b) {
                int c = Double.compare(b.score, a.score);
                if (c != 0) return c;
                c = Double.compare(a.angle, b.angle);
                if (c != 0) return c;
                c = Double.compare(a.center.x, b.center.x);
                if (c != 0) return c;
                return Double.compare(a.center.y, b.center.y);
            }
        });
    }

    private static final class Plan {
        final List<Placement> poses;
        final double score;
        Plan(List<Placement> poses, double score) {
            this.poses = poses;
            this.score = score;
        }
    }

    private static final class Candidate {
        final Item item;
        final Vec center;
        final double angle;
        final Obb fridgeZone;
        final double score;
        final boolean continuation;

        Candidate(Item item, Vec center, double angle, Obb fridgeZone, double score) {
            this(item, center, angle, fridgeZone, score, false);
        }

        Candidate(Item item, Vec center, double angle, Obb fridgeZone, double score,
                  boolean continuation) {
            this.item = item;
            this.center = center;
            this.angle = angle;
            this.fridgeZone = fridgeZone;
            this.score = score;
            this.continuation = continuation;
        }
    }
}
