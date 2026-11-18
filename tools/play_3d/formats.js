(function () {
  "use strict";

  const poseColumns = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"];
  const csvColumns = ["camera_ns", "valid", ...poseColumns];
  const decimalPattern = /^([+-]?)(\d*)(?:\.(\d*))?(?:[eE]([+-]?\d+))?$/;
  const ticksPerSecond = 1e15;

  function invalid(line, message) {
    throw new Error(`第 ${line} 行${message}`);
  }

  function timestamp(value, isNanoseconds, line) {
    const match = decimalPattern.exec(value);
    if (!match || !(match[2] || match[3])) invalid(line, "的时间戳格式有误。");
    const [, sign, whole, fraction = "", power = "0"] = match;
    const exponent = Number(power);
    if (!Number.isSafeInteger(exponent) || Math.abs(exponent) > 100 || whole.length + fraction.length > 100) {
      invalid(line, "的时间戳超出可处理范围。");
    }
    // Subtract timestamps as integers before converting the elapsed time to
    // Number, including sub-nanosecond decimals in native ORB-SLAM3 exports.
    let ticks = BigInt(whole + fraction);
    const shift = (isNanoseconds ? 6 : 15) + exponent - fraction.length;
    if (shift < 0) ticks /= 10n ** BigInt(-shift);
    else ticks *= 10n ** BigInt(shift);
    return sign === "-" ? -ticks : ticks;
  }

  function pose(values, line) {
    const numbers = values.map((value) => {
      const match = decimalPattern.exec(value);
      const result = Number(value);
      if (!match || !(match[2] || match[3]) || !Number.isFinite(result)) {
        invalid(line, "的有效位姿必须包含 7 个有限数值。");
      }
      return result;
    });
    const p = numbers.slice(0, 3);
    const quaternion = numbers.slice(3);
    const norm = Math.hypot(...quaternion);
    if (!Number.isFinite(norm) || norm === 0) invalid(line, "的四元数无效。");
    return {p, q: quaternion.map((value) => value / norm)};
  }

  function splitCsv(value, line) {
    const columns = [];
    const column = /[ \t]*(?:"((?:[^"]|"")*)"[ \t]*|([^",]*))(,|$)/y;
    let separator;
    do {
      const match = column.exec(value);
      if (!match) invalid(line, "的 CSV 引号或分隔符有误。");
      const [, quoted, unquoted, delimiter] = match;
      columns.push((quoted === undefined ? unquoted : quoted.replace(/""/g, '"')).trim());
      separator = delimiter;
    } while (separator === ",");
    return columns;
  }

  function parseCsv(records) {
    const header = splitCsv(records[0].value, records[0].line);
    if (new Set(header).size !== header.length) throw new Error("CSV 表头包含重复字段。");
    const missing = csvColumns.filter((key) => !header.includes(key));
    if (missing.length) throw new Error(`CSV 缺少字段 ${missing.join("、")}。`);

    return records.slice(1).map(({value, line}, rowIndex) => {
      const values = splitCsv(value, line);
      if (values.length !== header.length) invalid(line, "的 CSV 字段数与表头不一致。");
      const get = (key) => values[header.indexOf(key)];
      const valid = get("valid");
      if (valid !== "0" && valid !== "1") invalid(line, "的 valid 必须是 0 或 1。");
      const sourceIndex = get("frame_idx");
      const index = sourceIndex === undefined ? rowIndex : Number(sourceIndex);
      if (sourceIndex !== undefined && (!/^\d+$/.test(sourceIndex) || !Number.isSafeInteger(index))) {
        invalid(line, "的 frame_idx 必须是非负整数。");
      }
      return {
        stamp: timestamp(get("camera_ns"), true, line), line, index,
        map: get("map_id"), recovery: get("recovery_id"),
        ...(valid === "1" ? pose(poseColumns.map(get), line) : {p: null, q: null})
      };
    });
  }

  function parseText(records, isNanoseconds) {
    return records.map(({value, line}, index) => {
      const [time, ...values] = value.split(/\s+/);
      if (values.length !== 7) {
        invalid(line, "应包含时间戳和 7 个空格分隔的位姿值。");
      }
      return {stamp: timestamp(time, isNanoseconds, line), line, index, ...pose(values, line)};
    });
  }

  function medianInterval(times) {
    const intervals = times.slice(1).map((time, i) => time - times[i]).sort((a, b) => a - b);
    if (!intervals.length) return null;
    const center = (intervals.length - 1) / 2;
    return (intervals[Math.floor(center)] + intervals[Math.ceil(center)]) / 2;
  }

  function timeline(rows) {
    if (!rows.length) throw new Error("文件中没有位姿记录。");
    const times = rows.map((row, i) => {
      if (i && row.stamp <= rows[i - 1].stamp) invalid(row.line, "的时间戳必须严格递增。");
      if (i && row.index <= rows[i - 1].index) invalid(row.line, "的 frame_idx 必须严格递增。");
      return Number(row.stamp - rows[0].stamp) / ticksPerSecond;
    });
    const nominalInterval = medianInterval(times);
    const maxGap = nominalInterval === null ? Infinity : Math.max(0.1, 5 * nominalInterval);
    let segment = -1;
    const frames = rows.map((row, i) => {
      const t = times[i];
      if (i && t <= times[i - 1]) invalid(row.line, "的时间戳间隔小于播放器能表示的精度。");
      const previous = rows[i - 1];
      if (row.p !== null && (!previous || previous.p === null || row.index !== previous.index + 1 ||
          row.map !== previous.map || row.recovery !== previous.recovery || t - times[i - 1] > maxGap)) {
        segment += 1;
      }
      return {t, p: row.p, q: row.q, segment: row.p === null ? -1 : segment, index: row.index};
    });
    return {frames, nominalInterval};
  }

  function parse(text, format, name = "位姿轨迹") {
    const records = text.split(/\r\n|\n|\r/)
      .map((value, i) => ({value: value.trim(), line: i + 1}))
      .filter(({value}) => value && !value.startsWith("#"));
    if (!records.length) throw new Error("文件中没有位姿记录。");
    if (format === "auto") {
      if (!records[0].value.includes(",")) {
        throw new Error("请为文本轨迹选择时间戳单位（秒或纳秒）。");
      }
      format = "csv";
    }
    if (format === "csv") return {name, format, ...timeline(parseCsv(records))};
    if (format === "tum" || format === "ns") {
      return {name, format, ...timeline(parseText(records, format === "ns"))};
    }
    throw new Error("请选择受支持的位姿文件格式。");
  }

  globalThis.VioPoseIO = Object.freeze({parse});
})();
