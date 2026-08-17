import re
from pathlib import Path
from urllib.parse import unquote


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MARKDOWN_LINK = re.compile(r"\[[^]]+\]\(([^)]+)\)")
GATE_ID = re.compile(r"`((?:FS|CO|FB)-\d{2})`")


def _heading_slugs(document):
    headings = re.findall(r"^#{1,6}\s+(.+)$", document.read_text(), re.MULTILINE)
    return {
        re.sub(r"[^\w -]", "", heading.lower()).replace(" ", "-")
        for heading in headings
    }


def test_local_markdown_links_exist():
    documents = (
        PACKAGE_ROOT / "AGENTS.md",
        PACKAGE_ROOT / "README.md",
        *(PACKAGE_ROOT / "docs").rglob("*.md"),
        *(PACKAGE_ROOT / "document").rglob("*.md"),
    )
    for document in documents:
        for raw_target in MARKDOWN_LINK.findall(document.read_text()):
            target = raw_target.strip()
            if target.startswith(("http://", "https://")):
                continue
            if target.startswith("<") and ">" in target:
                target = target[1:target.index(">")]
            path_text, _, fragment = target.partition("#")
            linked_document = document.parent / unquote(path_text or document.name)
            assert linked_document.exists(), (
                f"{document.relative_to(PACKAGE_ROOT)}: missing link {path_text}"
            )
            if fragment and linked_document.suffix == ".md":
                assert unquote(fragment) in _heading_slugs(linked_document), (
                    f"{document.relative_to(PACKAGE_ROOT)}: missing anchor {fragment}"
                )


def test_every_acceptance_gate_has_a_verification_route():
    acceptance = (PACKAGE_ROOT / "docs/acceptance-contract.md").read_text()
    verification = (PACKAGE_ROOT / "docs/verification.md").read_text()
    gate_map = verification.split("## Gate map", 1)[1].split(
        "## Documentation", 1
    )[0]
    assert set(GATE_ID.findall(acceptance)) == set(GATE_ID.findall(gate_map))


def test_problem_index_matches_problem_files():
    problem_dir = PACKAGE_ROOT / "document/problem"
    index = (problem_dir / "README.md").read_text()
    indexed = set(re.findall(r"\]\((FT-\d{8}-\d{2}\.md)\)", index))
    indexed_status = dict(
        re.findall(
            r"\[`(FT-\d{8}-\d{2})`\]\([^)]*\).*\| `([A-Z_]+)` \|$",
            index,
            re.MULTILINE,
        )
    )
    files = {path.name for path in problem_dir.glob("FT-*.md")}
    assert indexed == files
    for path in problem_dir.glob("FT-*.md"):
        problem = path.read_text()
        assert problem.startswith(f"# {path.stem}:")
        assert indexed_status[path.stem] == re.search(
            r"^- 상태: ([A-Z_]+)$", problem, re.MULTILINE
        ).group(1)


def test_repository_map_routes_exist():
    repository_map = (PACKAGE_ROOT / "docs/repository-map.md").read_text()
    routes = repository_map.split("## Change routes", 1)[1].split(
        "## External boundaries", 1
    )[0]
    for target in re.findall(r"`([^`]+)`", routes):
        if target == "BUILD_TESTING":
            continue
        path = Path(target) if target.startswith("/") else PACKAGE_ROOT / target
        matches = list(path.parent.glob(path.name)) if "*" in path.name else [path]
        assert matches and all(match.exists() for match in matches), target
