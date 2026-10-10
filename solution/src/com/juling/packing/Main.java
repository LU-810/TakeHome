package com.juling.packing;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

/**
 * 程序入口。
 *
 * 用法：
 *   javac -encoding UTF-8 -d out src/com/juling/packing/*.java
 *   java  -cp out com.juling.packing.Main <json文件或目录> [更多...]
 * 不传参数时，处理当前目录下的 example*.json。
 * 结果（placement_*.json 与同名 svg）写入输入文件旁的 result/ 目录，并在控制台打印。
 */
public final class Main {

    public static void main(String[] args) throws IOException {
        List<Path> inputs = collectInputs(args);
        if (inputs.isEmpty()) {
            System.out.println("没有找到 example*.json 输入文件，请把 json 路径作为参数传入。");
            return;
        }

        for (Path file : inputs) {
            processOne(file);
        }
    }

    private static List<Path> collectInputs(String[] args) throws IOException {
        List<Path> result = new ArrayList<Path>();
        List<Path> roots = new ArrayList<Path>();
        if (args.length == 0) {
            roots.add(Paths.get(".").toAbsolutePath().normalize());
        } else {
            for (String a : args) roots.add(Paths.get(a).toAbsolutePath().normalize());
        }
        for (Path root : roots) {
            if (Files.isRegularFile(root)) {
                if (root.toString().toLowerCase().endsWith(".json")) result.add(root);
            } else if (Files.isDirectory(root)) {
                try (Stream<Path> walk = Files.walk(root, 1)) {
                    List<Path> found = new ArrayList<Path>();
                    walk.filter(Files::isRegularFile)
                        .filter(p -> p.getFileName().toString().toLowerCase().matches("example.*\\.json"))
                        .forEach(found::add);
                    Collections.sort(found);
                    result.addAll(found);
                }
            }
        }
        return result;
    }

    private static void processOne(Path file) throws IOException {
        String text = new String(Files.readAllBytes(file), StandardCharsets.UTF_8);
        Map<String, Object> root = Json.asMap(Json.parse(text));

        Vec[] boundary = parsePoints(root.get("boundary"));
        Vec[] doorPts = parsePoints(root.get("door"));
        if (doorPts.length != 2) {
            throw new IllegalArgumentException("door 必须是两个点");
        }
        boolean inward = Boolean.TRUE.equals(root.get("isOpenInward"));
        Map<String, Object> toPlace = Json.asMap(root.get("algoToPlace"));

        List<Item> items = new ArrayList<Item>();
        final List<String> inputOrder = new ArrayList<String>();
        for (Map.Entry<String, Object> e : toPlace.entrySet()) {
            List<Object> size = Json.asList(e.getValue());
            double length = Json.asDouble(size.get(0));
            double width = Json.asDouble(size.get(1));
            items.add(new Item(e.getKey(), length, width));
            inputOrder.add(e.getKey());
        }

        Room room = new Room(boundary, doorPts[0], doorPts[1], inward);
        PackingSolver solver = new PackingSolver(room, items);
        List<Placement> solved = solver.solve();

        String baseName = file.getFileName().toString().replaceAll("\\.json$", "");
        Path outDir = file.toAbsolutePath().getParent().resolve("result");
        Files.createDirectories(outDir);

        System.out.println("================================================================");
        System.out.println("输入文件: " + file);
        System.out.println("门宽 N = " + Math.round(room.doorWidth) + " mm，"
                + (inward ? "内开门（禁放 N×N = " + Math.round(room.doorWidth) + "×" + Math.round(room.doorWidth) + "）"
                          : "外开门（室内通行净深 " + Math.round(Room.DOOR_PASS_DEPTH) + "mm）"));
        System.out.println("物品摆放顺序（按面积降序）: " + solver.orderedItems());

        Map<String, Object> output = new LinkedHashMap<String, Object>();
        output.put("feasible", solved != null);
        output.put("source", file.getFileName().toString());
        output.put("doorWidth", Math.round(room.doorWidth));
        output.put("doorOpenInward", inward);

        if (solved == null) {
            System.out.println("结论: 不可行（在所有候选位姿与回溯搜索后仍无法放下全部物品）");
            output.put("placements", Collections.emptyList());
        } else {
            System.out.println("结论: 可行。各物品中心点与旋转角度（0° 时 length 边沿 x 轴）:");
            Map<String, Placement> byName = new LinkedHashMap<String, Placement>();
            for (Placement p : solved) byName.put(p.item.name, p);

            List<Object> arr = new ArrayList<Object>();
            for (String name : inputOrder) {
                Placement p = byName.get(name);
                Map<String, Object> po = new LinkedHashMap<String, Object>();
                po.put("item", name);
                po.put("type", p.item.type.name().toLowerCase());
                po.put("length", Math.round(p.item.length));
                po.put("width", Math.round(p.item.width));
                List<Object> center = new ArrayList<Object>();
                center.add(round2(p.center.x));
                center.add(round2(p.center.y));
                po.put("center", center);
                po.put("angleDeg", round2(p.angleDeg));
                arr.add(po);
                System.out.printf("  %-12s 中心=(%11.2f, %11.2f)  角度=%6.2f°%n",
                        name, p.center.x, p.center.y, p.angleDeg);
            }
            output.put("placements", arr);
        }

        Path jsonOut = outDir.resolve("placement_" + baseName + ".json");
        Files.write(jsonOut, Json.write(output, 2).getBytes(StandardCharsets.UTF_8));

        String svg = SvgWriter.write(room, solved,
                baseName + "  摆放结果：" + (solved != null ? "可行" : "不可行"));
        Path svgOut = outDir.resolve(baseName + ".svg");
        Files.write(svgOut, svg.getBytes(StandardCharsets.UTF_8));

        if (solved != null) {
            Path reachOut = outDir.resolve(baseName + "_reach.svg");
            Reachability.writeSvg(reachOut.toString(), room, solved);
            System.out.println("通道可达性图已写出: " + reachOut);
        }

        System.out.println("结果已写出: " + jsonOut);
        System.out.println("平面图已写出: " + svgOut);
        System.out.println();
    }

    private static Vec[] parsePoints(Object arr) {
        List<Object> list = Json.asList(arr);
        Vec[] pts = new Vec[list.size()];
        for (int i = 0; i < list.size(); i++) {
            List<Object> xy = Json.asList(list.get(i));
            pts[i] = new Vec(Json.asDouble(xy.get(0)), Json.asDouble(xy.get(1)));
        }
        return pts;
    }

    private static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }
}
