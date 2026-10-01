#!/usr/bin/env python3
"""校验 Gazebo 模型目录（classic / Sim 通用）。

用法:
    ./check-models.py [模型根目录 ...]        # 默认 gz-cache/models
    ./check-models.py --fix-version <dir>     # 给缺失的 <sdf> 补上 version 属性

检查项:
  1. model.config 能否解析、是否有 <name> / <sdf>
  2. <sdf> 是否带 version 属性（缺失时 gz sim 会报
     "Can not find the XML attribute 'version' in sdf XML tag"）
  3. <sdf> 指向的文件是否存在
  4. 该文件自身的 <sdf version="..."> 与 model.config 声明是否一致
  5. SDF 内所有 <uri> 能否解析:
       model://<name>/<rel>  -> <model_dir>/<rel>
       model://<name>        -> 同级模型目录
       file://...            -> 宿主路径
       http(s):// / package:// -> 跳过（远端 / ROS 包）
  6. 是否使用 classic Gazebo 材质脚本（Gazebo Sim 无法渲染，会回退默认材质）
"""

from __future__ import annotations

import argparse
import re
import sys
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path

ERROR = "ERROR"
WARN = "WARN"
INFO = "INFO"

SDF_VERSION_RE = re.compile(r"^<sdf\s+version=['\"]([^'\"]+)['\"]", re.IGNORECASE)


def localname(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


URI_ELEMENTS = {"uri", "normal_map", "normal", "diffuse", "specular", "emissive",
                "albedo", "roughness", "metalness", "light_map", "environment"}


def is_uri_value(tag: str, text: str) -> bool:
    """<uri> is always a URI; the PBR texture elements hold URIs too, but e.g.
    <ambient> holds a colour vector, so require an explicit scheme."""
    return tag == "uri" or "://" in text


def iter_uris(path: Path):
    """Yield every URI-bearing value inside an SDF/URDF file."""
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        raise ValueError(f"XML 解析失败: {exc}") from exc
    for elem in tree.iter():
        tag = localname(elem.tag)
        text = (elem.text or "").strip()
        if tag in URI_ELEMENTS and text and is_uri_value(tag, text):
            yield text


def declared_sdf_version(path: Path) -> str | None:
    """Version from the <sdf version='...'> root attribute of an SDF file."""
    try:
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            head = fh.read(4096)
    except OSError:
        return None
    match = SDF_VERSION_RE.search(head.lstrip())
    return match.group(1) if match else None


def build_index(root: Path) -> tuple[dict[str, Path], dict[str, Path]]:
    """Map both directory names and <name> values to model directories."""
    by_dir: dict[str, Path] = {}
    by_name: dict[str, Path] = {}
    for config in sorted(root.glob("*/model.config")):
        model_dir = config.parent
        by_dir[model_dir.name] = model_dir
        try:
            tree = ET.parse(config)
        except ET.ParseError:
            for match in re.finditer(r"<name>([^<]+)</name>", config.read_text("utf-8", "replace")):
                by_name.setdefault(match.group(1).strip(), model_dir)
            continue
        for elem in tree.iter():
            if localname(elem.tag) == "name" and elem.text:
                by_name.setdefault(elem.text.strip(), model_dir)
    return by_dir, by_name


def uri_findings(text: str, label: str) -> list[tuple[str, str]]:
    """Detect XML constructs that Gazebo's tinyxml2 rejects outright.

    Verified against gz-sim 8 behaviour:
      * a comment (non-whitespace) before <?xml -> XML_ERROR_PARSING_DECLARATION
      * an unquoted attribute value             -> XML_ERROR_PARSING_ATTRIBUTE
    Leading whitespace before the declaration is tolerated by tinyxml2.
    """
    issues: list[tuple[str, str]] = []

    stripped_bom = text.lstrip("\ufeff")
    decl_at = stripped_bom.find("<?xml")
    if decl_at > 0:
        prefix = stripped_bom[:decl_at]
        if prefix.strip():
            issues.append((ERROR, f"{label}: XML 声明前有注释/内容 -> tinyxml2 报 "
                                  f"XML_ERROR_PARSING_DECLARATION，模型无法加载"))
        else:
            issues.append((WARN, f"{label}: XML 声明前有空白（严格 XML 非法，Gazebo 可容忍）"))

    # 去掉注释后再找未加引号的属性，避免把注释正文当属性
    no_comments = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    for tag in re.finditer(r"<[A-Za-z_][^<>]*>", no_comments):
        for attr in re.finditer(r"""[A-Za-z_:][\w:.-]*\s*=\s*([^\s"'>=/][^\s>]*)""", tag.group(0)):
            line = no_comments.count("\n", 0, tag.start()) + 1
            issues.append((ERROR, f"{label}: 属性值未加引号 [{attr.group(0).strip()}] "
                                  f"(第 {line} 行) -> tinyxml2 报 XML_ERROR_PARSING_ATTRIBUTE"))
    return issues


def extract_uris_lenient(text: str) -> list[str]:
    """Pull URI-bearing values out of a file that strict XML parsing cannot handle."""
    no_comments = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    found = []
    for match in re.finditer(r"<([A-Za-z_][\w:.-]*)>\s*([^<]*?)\s*</\1>", no_comments):
        tag, value = match.group(1), match.group(2)
        if tag in URI_ELEMENTS and value and is_uri_value(tag, value):
            found.append(value)
    return found


PLUGIN_RE = re.compile(r"""<plugin\b[^>]*?\bfilename\s*=\s*["']([^"']*\.so)["']"""
                       r"""|<filename>\s*([^<]*\.so)\s*</filename>""")
SCRIPT_RE = re.compile(r"<script>(.*?)</script>", re.S)


def legacy_findings(text: str, label: str) -> list[tuple[str, str]]:
    """Find classic-Gazebo constructs that Gazebo Sim silently drops."""
    issues: list[tuple[str, str]] = []
    no_comments = re.sub(r"<!--.*?-->", "", text, flags=re.S)

    for match in PLUGIN_RE.finditer(no_comments):
        lib = match.group(1) or match.group(2)
        issues.append((WARN, f"{label}: 引用 classic 插件 [{lib}]，"
                             f"gz sim 加载不了（功能会静默丢失）"))

    for match in SCRIPT_RE.finditer(no_comments):
        if "<uri>" not in match.group(1):
            line = no_comments.count("\n", 0, match.start()) + 1
            issues.append((WARN, f"{label}: <script> 缺少子元素 <uri>（第 {line} 行）"))
    return issues


def check_uri(uri: str, model_dir: Path, root: Path, index) -> tuple[str, str] | None:
    by_dir, by_name = index
    if uri.startswith(("http://", "https://", "package://", "spawn://")):
        return None

    if uri.startswith("file://"):
        target = Path(urllib.parse.unquote(uri[len("file://") :]))
        if target.exists():
            return None
        # classic Gazebo 的 <material><script><uri> 用相对 media 路径的 file:// URI，
        # sdformat 不会去解析它，只导致材质回退，属外观噪音。
        if "media/materials/scripts" in uri:
            return (INFO, f"classic 材质脚本（gz sim 会回退默认材质）: {target.name}")
        # 其它 file:// URI（贴图、法线贴图、网格）会被真正解析，
        # 解析不到会让整个模型加载失败。
        return (ERROR, f"file:// 目标不存在，模型会加载失败: {uri}")

    if not uri.startswith("model://"):
        return (WARN, f"无法识别的 URI: {uri}")

    rest = urllib.parse.unquote(uri[len("model://") :]).strip("/")
    model_name, _, rel = rest.partition("/")

    if not rel:
        if model_name in by_dir or model_name in by_name:
            return None
        return (WARN, f"引用外部模型 model://{model_name}（不在本目录内）")

    if model_name in {model_dir.name}:
        target = model_dir / rel
    else:
        owner = by_dir.get(model_name) or by_name.get(model_name)
        if owner is None:
            return (WARN, f"引用未知模型 model://{model_name}/...")
        target = owner / rel

    return None if target.exists() else (ERROR, f"引用的文件不存在: model://{rest}")


def check_model(model_dir: Path, root: Path, index) -> list[tuple[str, str, str]]:
    issues: list[tuple[str, str, str]] = []

    def add(level: str, msg: str) -> None:
        issues.append((level, model_dir.name, msg))

    config = model_dir / "model.config"
    if not config.is_file():
        add(ERROR, "缺少 model.config")
        return issues

    config_text = config.read_text(encoding="utf-8", errors="replace")
    issues.extend((level, model_dir.name, msg) for level, msg in uri_findings(config_text, "model.config"))

    name_text = sdf_filename = ""
    declared = ""
    parsed = False
    try:
        tree = ET.parse(config)
        parsed = True
    except ET.ParseError as exc:
        add(WARN, f"model.config 严格解析失败（Gazebo 可能仍接受）: {exc}")

    if parsed:
        name_elem = sdf_elem = None
        for elem in tree.iter():
            tag = localname(elem.tag)
            if tag == "name" and name_elem is None:
                name_elem = elem
            elif tag == "sdf" and sdf_elem is None:
                sdf_elem = elem
        if name_elem is None or not (name_elem.text or "").strip():
            add(ERROR, "model.config 缺少 <name>")
        if sdf_elem is None or not (sdf_elem.text or "").strip():
            add(ERROR, "model.config 缺少 <sdf> 文件名")
            return issues
        name_text = (name_elem.text or "").strip() if name_elem is not None else ""
        sdf_filename = (sdf_elem.text or "").strip()
        declared = (sdf_elem.get("version") or "").strip()
    else:
        match = re.search(r"<sdf([^>]*)>([^<]+)</sdf>", config_text)
        if not match:
            add(ERROR, "model.config 缺少 <sdf> 文件名")
            return issues
        sdf_filename = match.group(2).strip()
        version_attr = re.search(r"""version\s*=\s*["']([^"']+)["']""", match.group(1))
        declared = version_attr.group(1) if version_attr else ""
        name_match = re.search(r"<name>([^<]*)</name>", config_text)
        name_text = name_match.group(1).strip() if name_match else ""

    sdf_file = model_dir / sdf_filename
    if not sdf_file.is_file():
        add(ERROR, f"<sdf> 指向的文件不存在: {sdf_filename}")
        return issues

    actual = declared_sdf_version(sdf_file)
    if not declared:
        add(WARN, f"<sdf> 缺少 version 属性（文件实际为 {actual or '未知'}）")
    elif actual and declared != actual:
        add(WARN, f"version 不一致: model.config={declared}, {sdf_filename}={actual}")

    sdf_text = sdf_file.read_text(encoding="utf-8", errors="replace")
    issues.extend((level, model_dir.name, msg) for level, msg in uri_findings(sdf_text, sdf_file.name))

    try:
        uris = list(iter_uris(sdf_file))
    except ValueError as exc:
        add(WARN, f"{sdf_file.name} 严格解析失败（Gazebo 可能仍接受）: {exc}")
        uris = extract_uris_lenient(sdf_text)

    for uri in uris:
        result = check_uri(uri, model_dir, root, index)
        if result:
            add(*result)

    if "media/materials/scripts" in sdf_text:
        add(INFO, "使用 classic Gazebo 材质脚本 -> gz sim 会回退默认材质（外观问题）")

    issues.extend((level, model_dir.name, msg) for level, msg in legacy_findings(sdf_text, sdf_file.name))

    return issues


ATTR_RE = re.compile(r"""(?P<name>[A-Za-z_:][\w:.-]*)\s*=\s*(?P<value>[^\s"'>=/][^\s>]*)""")


def mask_comments(text: str) -> str:
    """Blank out comments while preserving offsets, so tag positions stay valid."""
    return re.sub(r"<!--.*?-->", lambda m: " " * (m.end() - m.start()), text, flags=re.S)


def xml_repairs(text: str) -> list[tuple[int, int, str]]:
    """Return (start, end, replacement) edits that make the file tinyxml2-safe."""
    edits: list[tuple[int, int, str]] = []
    masked = mask_comments(text)

    # 1) 删除不在文件开头（注释之后）的 XML 声明
    bom_offset = len(text) - len(text.lstrip("\ufeff"))
    decl_at = text.find("<?xml", bom_offset)
    if decl_at > 0 and text[bom_offset:decl_at].strip():
        end = text.find("?>", decl_at)
        if end != -1:
            end += 2
            while end < len(text) and text[end] in "\r\n":
                end += 1
            edits.append((decl_at, end, ""))

    # 2) 给未加引号的属性值补上双引号
    for tag in re.finditer(r"<[A-Za-z_][^<>]*>", masked):
        inner = tag.group(0)
        for match in ATTR_RE.finditer(inner):
            if match.start() == 0 or not inner[match.start() - 1].isspace():
                continue
            start = tag.start() + match.start("value")
            end = tag.start() + match.end("value")
            edits.append((start, end, f'"{match.group("value")}"'))

    return sorted(edits, key=lambda e: e[0])


def fix_xml(root: Path, models: list[Path]) -> int:
    """Rewrite models whose XML tinyxml2 would reject."""
    fixed = 0
    for model_dir in models:
        for candidate in sorted(model_dir.glob("*.sdf")) + [model_dir / "model.config"]:
            if not candidate.is_file():
                continue
            text = candidate.read_text(encoding="utf-8", errors="replace")
            edits = xml_repairs(text)
            if not edits:
                continue
            for start, end, replacement in reversed(edits):
                text = text[:start] + replacement + text[end:]
            candidate.write_text(text, encoding="utf-8")
            fixed += 1
    return fixed


def collect_models(root: Path) -> tuple[list[Path], list[Path]]:
    models, broken = [], []
    for child in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith(".")):
        if (child / "model.config").is_file():
            models.append(child)
        else:
            broken.append(child)
    return models, broken


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("roots", nargs="*", help="模型根目录，默认 my-slam/gz-cache/models")
    parser.add_argument("--fix-version", action="store_true", help="为缺失 version 的 <sdf> 补上文件自身的版本")
    parser.add_argument("--fix-xml", action="store_true", help="修复 tinyxml2 会拒绝的 XML（声明位置、未加引号的属性）")
    args = parser.parse_args()

    default_root = Path(__file__).resolve().parent / "gz-cache" / "models"
    roots = [Path(r).resolve() for r in args.roots] or [default_root]

    total_errors = 0
    for root in roots:
        print(f"=== {root} ===")
        if not root.is_dir():
            print(f"  {ERROR}: 目录不存在")
            total_errors += 1
            continue

        index = build_index(root)
        models, orphan_dirs = collect_models(root)
        print(f"  模型数: {len(models)}，无 model.config 的目录: {len(orphan_dirs)}")

        all_issues: list[tuple[str, str, str]] = []
        for model_dir in models:
            all_issues.extend(check_model(model_dir, root, index))

        by_level: dict[str, list[tuple[str, str]]] = {ERROR: [], WARN: [], INFO: []}
        for level, model, msg in all_issues:
            by_level[level].append((model, msg))

        for level in (ERROR, WARN, INFO):
            raw = by_level[level]
            if not raw:
                continue
            # 同一个模型里重复出现的问题合并计数（例如同一材质脚本被引用 7 次）
            counted: dict[tuple[str, str], int] = {}
            for model, msg in raw:
                counted[(model, msg)] = counted.get((model, msg), 0) + 1
            print(f"  {level} ({len(raw)} 处, {len(counted)} 类):")
            for (model, msg), count in sorted(counted.items())[:200]:
                suffix = f" ×{count}" if count > 1 else ""
                print(f"    - {model}: {msg}{suffix}")
            if len(counted) > 200:
                print(f"    ... 其余 {len(counted) - 200} 类省略")

        total_errors += len(by_level[ERROR])

        if args.fix_version:
            fixed = fix_versions(root, models)
            print(f"  已修正 <sdf version>: {fixed} 个")
        if args.fix_xml:
            fixed = fix_xml(root, models)
            print(f"  已修正 XML 文件: {fixed} 个")
        print()

    return 1 if total_errors else 0


def fix_versions(root: Path, models: list[Path]) -> int:
    """Add the version attribute to <sdf> entries that lack one.

    Uses a targeted text substitution so the rest of model.config keeps its
    original formatting (and the diff stays a single line).
    """
    fixed = 0
    for model_dir in models:
        config = model_dir / "model.config"
        try:
            tree = ET.parse(config)
        except ET.ParseError:
            continue

        wanted: dict[str, str] = {}
        for elem in tree.iter():
            if localname(elem.tag) != "sdf" or (elem.get("version") or "").strip():
                continue
            filename = (elem.text or "").strip()
            version = declared_sdf_version(model_dir / filename)
            if filename and version:
                wanted[filename] = version
        if not wanted:
            continue

        text = config.read_text(encoding="utf-8")
        for filename, version in wanted.items():
            pattern = re.compile(
                r"<sdf(\s*)>(" + re.escape(filename) + r"</sdf>)",
            )
            text, count = pattern.subn(rf'<sdf version="{version}"\1>\2', text)
            if not count:
                # attribute already present on that particular tag
                continue
        config.write_text(text, encoding="utf-8")
        fixed += 1
    return fixed


if __name__ == "__main__":
    sys.exit(main())
