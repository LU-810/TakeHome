package com.juling.packing;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * 一个零依赖的迷你 JSON 解析器 / 输出器，仅覆盖本题需要的语法：
 * 对象（保持插入顺序）、数组、字符串、数字（按 double 解析）、true/false/null。
 */
public final class Json {

    // ---------------- 解析 ----------------

    private final String s;
    private int i;

    private Json(String s) {
        this.s = s;
    }

    public static Object parse(String text) {
        Json j = new Json(text);
        j.skipWs();
        Object v = j.readValue();
        j.skipWs();
        if (j.i < j.s.length()) {
            throw new IllegalArgumentException("JSON 解析失败，尾部存在多余字符，位置 " + j.i);
        }
        return v;
    }

    private Object readValue() {
        skipWs();
        if (i >= s.length()) throw new IllegalArgumentException("JSON 提前结束");
        char ch = s.charAt(i);
        if (ch == '{') return readObject();
        if (ch == '[') return readArray();
        if (ch == '"') return readString();
        if (ch == 't' || ch == 'f') return readBoolean();
        if (ch == 'n') { readLiteral("null"); return null; }
        return readNumber();
    }

    private Map<String, Object> readObject() {
        Map<String, Object> map = new LinkedHashMap<String, Object>();
        expect('{');
        skipWs();
        if (peek() == '}') { i++; return map; }
        while (true) {
            skipWs();
            String key = readString();
            skipWs();
            expect(':');
            Object val = readValue();
            map.put(key, val);
            skipWs();
            char c = next();
            if (c == ',') continue;
            if (c == '}') break;
            throw new IllegalArgumentException("JSON 对象中应为 ',' 或 '}'，实际为 " + c);
        }
        return map;
    }

    private List<Object> readArray() {
        List<Object> list = new ArrayList<Object>();
        expect('[');
        skipWs();
        if (peek() == ']') { i++; return list; }
        while (true) {
            list.add(readValue());
            skipWs();
            char c = next();
            if (c == ',') continue;
            if (c == ']') break;
            throw new IllegalArgumentException("JSON 数组中应为 ',' 或 ']'，实际为 " + c);
        }
        return list;
    }

    private String readString() {
        expect('"');
        StringBuilder sb = new StringBuilder();
        while (true) {
            char c = next();
            if (c == '"') break;
            if (c == '\\') {
                char e = next();
                switch (e) {
                    case '"': sb.append('"'); break;
                    case '\\': sb.append('\\'); break;
                    case '/': sb.append('/'); break;
                    case 'b': sb.append('\b'); break;
                    case 'f': sb.append('\f'); break;
                    case 'n': sb.append('\n'); break;
                    case 'r': sb.append('\r'); break;
                    case 't': sb.append('\t'); break;
                    case 'u':
                        String hex = s.substring(i, i + 4);
                        i += 4;
                        sb.append((char) Integer.parseInt(hex, 16));
                        break;
                    default: throw new IllegalArgumentException("非法转义: \\" + e);
                }
            } else {
                sb.append(c);
            }
        }
        return sb.toString();
    }

    private Double readNumber() {
        int start = i;
        while (i < s.length()) {
            char c = s.charAt(i);
            if (c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E' || (c >= '0' && c <= '9')) {
                i++;
            } else break;
        }
        return Double.valueOf(s.substring(start, i));
    }

    private Boolean readBoolean() {
        if (peek() == 't') { readLiteral("true"); return Boolean.TRUE; }
        readLiteral("false");
        return Boolean.FALSE;
    }

    private void readLiteral(String lit) {
        if (!s.regionMatches(i, lit, 0, lit.length())) {
            throw new IllegalArgumentException("JSON 字面量错误，位置 " + i);
        }
        i += lit.length();
    }

    private void expect(char c) {
        if (i >= s.length() || s.charAt(i) != c) {
            throw new IllegalArgumentException("JSON 期望 '" + c + "'，位置 " + i);
        }
        i++;
    }

    private char next() {
        if (i >= s.length()) throw new IllegalArgumentException("JSON 提前结束");
        return s.charAt(i++);
    }

    private char peek() {
        return i >= s.length() ? '\0' : s.charAt(i);
    }

    private void skipWs() {
        while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
    }

    // ---------------- 输出 ----------------

    /** 将 Map/List/字符串/数字等结构序列化为带缩进的 JSON 文本 */
    @SuppressWarnings("unchecked")
    public static String write(Object value, int indent) {
        StringBuilder sb = new StringBuilder();
        writeValue(sb, value, indent, 0);
        sb.append('\n');
        return sb.toString();
    }

    @SuppressWarnings("unchecked")
    private static void writeValue(StringBuilder sb, Object v, int indent, int depth) {
        if (v == null) {
            sb.append("null");
        } else if (v instanceof String) {
            sb.append('"').append(escape((String) v)).append('"');
        } else if (v instanceof Boolean || v instanceof Integer || v instanceof Long) {
            sb.append(v);
        } else if (v instanceof Double || v instanceof Float) {
            double d = ((Number) v).doubleValue();
            if (d == Math.rint(d) && !Double.isInfinite(d)) {
                sb.append((long) d);
            } else {
                sb.append(String.format(java.util.Locale.ROOT, "%.2f", d));
            }
        } else if (v instanceof Map) {
            writeObject(sb, (Map<String, Object>) v, indent, depth);
        } else if (v instanceof List) {
            writeArray(sb, (List<Object>) v, indent, depth);
        } else {
            sb.append('"').append(escape(String.valueOf(v))).append('"');
        }
    }

    private static void writeObject(StringBuilder sb, Map<String, Object> map, int indent, int depth) {
        if (map.isEmpty()) { sb.append("{}"); return; }
        sb.append('{');
        boolean first = true;
        String pad = indent > 0 ? repeat(' ', indent * (depth + 1)) : "";
        String endPad = indent > 0 ? repeat(' ', indent * depth) : "";
        for (Map.Entry<String, Object> e : map.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            if (indent > 0) sb.append('\n').append(pad);
            sb.append('"').append(escape(e.getKey())).append('"');
            sb.append(indent > 0 ? ": " : ":");
            writeValue(sb, e.getValue(), indent, depth + 1);
        }
        if (indent > 0) sb.append('\n').append(endPad);
        sb.append('}');
    }

    private static void writeArray(StringBuilder sb, List<Object> list, int indent, int depth) {
        if (list.isEmpty()) { sb.append("[]"); return; }
        sb.append('[');
        boolean first = true;
        String pad = indent > 0 ? repeat(' ', indent * (depth + 1)) : "";
        String endPad = indent > 0 ? repeat(' ', indent * depth) : "";
        for (Object o : list) {
            if (!first) sb.append(',');
            first = false;
            if (indent > 0) sb.append('\n').append(pad);
            writeValue(sb, o, indent, depth + 1);
        }
        if (indent > 0) sb.append('\n').append(endPad);
        sb.append(']');
    }

    private static String escape(String s) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default: sb.append(c);
            }
        }
        return sb.toString();
    }

    private static String repeat(char c, int n) {
        StringBuilder sb = new StringBuilder(n);
        for (int k = 0; k < n; k++) sb.append(c);
        return sb.toString();
    }

    // ---------------- 类型安全取值 ----------------

    @SuppressWarnings("unchecked")
    public static Map<String, Object> asMap(Object o) {
        return (Map<String, Object>) o;
    }

    @SuppressWarnings("unchecked")
    public static List<Object> asList(Object o) {
        return (List<Object>) o;
    }

    public static double asDouble(Object o) {
        return ((Number) o).doubleValue();
    }
}
