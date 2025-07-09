# Claude Code Guidelines by Sabrina Ramonov (Python/ML Adaptation)

## Implementation Best Practices

### 0 — Purpose

These rules ensure maintainability, safety, and developer velocity.
**MUST** rules are enforced by CI; **SHOULD** rules are strongly recommended.

---

### 1 — Before Coding

- **BP-1 (MUST)** Ask the user clarifying questions.
- **BP-2 (SHOULD)** Draft and confirm an approach for complex work.
- **BP-3 (SHOULD)** If ≥ 2 approaches exist, list clear pros and cons.

---

### 2 — While Coding

- **C-1 (MUST)** Follow TDD: scaffold stub -> write failing test -> implement.
- **C-2 (MUST)** Name functions using established domain vocabulary for consistency.
- **C-3 (SHOULD NOT)** Introduce classes when small testable functions suffice.
- **C-4 (SHOULD)** Prefer simple, composable, testable functions.
- **C-5 (MUST)** Use `NewType` or `Annotated` for ID types instead of raw strings or ints.
  ```python
  from typing import NewType

  UserId = NewType('UserId', str)   # ✅ Good
  UserId = str                      # ❌ Bad
  ```
- **C-6 (MUST)** Use explicit type-only imports where applicable (e.g. `if TYPE_CHECKING`).
- **C-7 (SHOULD NOT)** Add comments except for critical caveats; rely on self-explanatory code.
- **C-8 (SHOULD)** Default to `TypedDict` or dataclasses; use inheritance only when necessary.
- **C-9 (SHOULD NOT)** Extract a new function unless it will be reused, makes logic testable, or meaningfully improves clarity.

---

### 3 — Testing

- **T-1 (MUST)** For a simple function, colocate unit tests in `test_*.py` in the same directory.
- **T-2 (MUST)** For any API or I/O change, add/extend integration tests under `tests/integration/`.
- **T-3 (MUST)** ALWAYS separate pure logic unit tests from I/O or DB-dependent integration tests.
- **T-4 (SHOULD)** Prefer integration tests over mocking-heavy tests.
- **T-5 (SHOULD)** Unit-test complex algorithms thoroughly.
- **T-6 (SHOULD)** Use expressive, minimal assertions:
  ```python
  assert result == [value]  # ✅ Good

  assert len(result) == 1   # ❌ Bad
  assert result[0] == value # ❌ Bad
  ```

---

### 4 — Database

- **D-1 (MUST)** Annotate DB helpers with `Session | SessionTransaction` (e.g. SQLAlchemy types).
- **D-2 (SHOULD)** Manually override incorrect or unsafe ORM type inferences using Pydantic or `Annotated`.

---

### 5 — Code Organization

- **O-1 (MUST)** Place shared code in `common/` or `shared/` modules only if reused by ≥ 2 packages or services.

---

### 6 — Tooling Gates

- **G-1 (MUST)** `black --check .` passes.
- **G-2 (MUST)** `mypy`, `ruff`, and `pytest` all pass without errors.

---

### 7 - Git

- **GH-1 (MUST)** Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0) format.
- **GH-2 (SHOULD NOT)** Mention Claude or Anthropic in commit messages.

---

## Writing Functions Best Practices

When evaluating whether a function you implemented is good or not, use this checklist:

1. Can you read the function and HONESTLY easily follow what it's doing? If yes, stop here.
2. Is cyclomatic complexity high (deep nesting, multiple branches)? Simplify if so.
3. Can a known algorithm or structure (e.g., heap, stack, trie, queue) make it clearer?
4. Any unused parameters?
5. Any unnecessary casting or coercion that could be handled by type annotations?
6. Is the function easily testable without mocking DB, Redis, or network calls?
7. Any hidden, untested dependencies? Should they be passed explicitly as arguments?
8. Brainstorm 3 better names—is the current name consistent and clear?

### Important:

DO NOT refactor into a separate function unless:

- It’s used in multiple places
- It enables unit-testing otherwise untestable logic
- The original is unreadable without excessive inline comments

---

## Writing Tests Best Practices

Use this checklist for test quality:

1. SHOULD parametrize inputs using `pytest.mark.parametrize`; avoid unexplained literals.
2. SHOULD NOT write trivial asserts (e.g., `assert 2 == 2`).
3. SHOULD align test description with what is asserted.
4. SHOULD verify output against independent expectation or properties, NOT the same logic reused.
5. SHOULD follow linting, typing, and formatting rules (e.g., `ruff`, `mypy`, `black`).
6. SHOULD express invariants when possible using `hypothesis`:
   ```python
   from hypothesis import given, strategies as st

   @given(st.text(), st.text())
   def test_character_count_concat(a, b):
       assert get_character_count(a + b) == get_character_count(a) + get_character_count(b)
   ```
7. Group unit tests with `class TestFunctionName:` or under `def describe_` functions.
8. Use `ANY` (from `unittest.mock`) when inputs or outputs can vary but should exist.
9. ALWAYS use strong, specific assertions (`assert x == 1`) over weak ones (`assert x >= 1`).
10. SHOULD test:
    - edge cases
    - realistic and malformed inputs
    - type boundaries
11. SHOULD NOT test things enforced by static typing.

---

## Code Organization

- `api/` - FastAPI backend service
  - `api/publishers/*.py` - Implementation of platform-specific publishing logic
- `frontend/` - React frontend (Next.js or other)
- `shared/` - Reusable Python logic or types (`dataclasses`, utility functions, schemas)
  - `shared/social.py` - Social media-specific character limits, validations
- `schemas/` - Pydantic or TypedDict schemas defining I/O contract

---

## Remember Shortcuts

### QNEW

When I type "qnew", this means:

```
Understand all BEST PRACTICES listed in CLAUDE.md.
Your code SHOULD ALWAYS follow these best practices.
```

---

### QPLAN

When I type "qplan", this means:

```
Analyze similar parts of the codebase and determine whether your plan:
- is consistent with rest of codebase
- introduces minimal changes
- reuses existing code
```

---

### QCODE

When I type "qcode", this means:

```
Implement your plan and make sure your new tests pass.
Always run tests to make sure you didn't break anything else.
Always run `black`, `ruff`, and `mypy` on changed files.
```

---

### QCHECK

When I type "qcheck", this means:

```
You are a SKEPTICAL senior software engineer.
Perform this analysis for every MAJOR code change you introduced (skip minor changes):

1. CLAUDE.md checklist Writing Functions Best Practices.
2. CLAUDE.md checklist Writing Tests Best Practices.
3. CLAUDE.md checklist Implementation Best Practices.
```

---

### QCHECKF

When I type "qcheckf", this means:

```
You are a SKEPTICAL senior software engineer.
Perform this analysis for every MAJOR function you added or edited (skip minor changes):

1. CLAUDE.md checklist Writing Functions Best Practices.
```

---

### QCHECKT

When I type "qcheckt", this means:

```
You are a SKEPTICAL senior software engineer.
Perform this analysis for every MAJOR test you added or edited (skip minor changes):

1. CLAUDE.md checklist Writing Tests Best Practices.
```

---

### QUX

When I type "qux", this means:

```
Imagine you are a human UX tester of the feature you implemented.
Output a comprehensive list of scenarios you would test, sorted by highest priority.
```

---

### QGIT

When I type "qgit", this means:

```
Add all changes to staging, create a commit, and push to remote.

Follow this checklist for writing your commit message:
- SHOULD use Conventional Commits format: https://www.conventionalcommits.org/en/v1.0.0
- SHOULD NOT refer to Claude or Anthropic in the commit message.
- SHOULD structure commit message as follows:
<type>[optional scope]: <description>
[optional body]
[optional footer(s)]
```
