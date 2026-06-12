# Autopilot Lite for C++20

## Principle

Light, fast, performant multirotor autopilot for C++20. This is meant to be a
mid-2020s replacement of
[mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers) for
offboard control of PX4 and Ardupilot drones.

## Target features

- A PID-based rate controller that combines the best features of PX4, Ardupilot,
  and Betaflight, and tidies them up into a single opinionated implementation
  (i.e., no edge-case handling)

- A position controller that achieves PX4/Ardupilot's smooth and robust waypoint
  navigation. It should do something more than point-wise tracking with a PD
  controller but NOT invole a full-blown polynomial trajectory generator.

## Structure

`apl20` for pure math kernels, `apl20_ros` for ROS integration. The former
should be reusable in non-ROS contexts. The latter finds the former via
`find_package` and adds ROS-specific glue

## Building

Use `just` as far as possible, but the underlying build system is CMake.
`just build` builds both `apl20` and `apl20_ros`.

## Style

- C++20.
- Google style. `PascalCase` for free functions, `camelCase` for class methods.
  Exception: MATLAB-isms like `rad2deg`, `wrapToPi`.
- All forms of metaprogramming are allowed.
- `.hpp` for C++-only headers. System headers in `<...>`, everything else in
  `"..."` (matches clang-format / Google sort order). Eigen headers count as
  library headers, not system headers.
- Always brace if/else/for/while bodies. Never `std::endl` (use `'\n'`). Never
  declare multiple variables in one statement.
- `#ifndef` guards, not `#pragma once`.
- Flat `include/<namespace>/header.hpp` layout. Subdirectories only when
  > 3 files in each and ≤5 left in the parent.
- Reusable math is header-only and takes Eigen by
  `const Eigen::MatrixBase<Derived>&`.
- At the math↔logic boundary, use `Eigen::Ref` (`const Eigen::Ref<const T>&` for
  reads, `Eigen::Ref<T>` for writes) — binds to both expressions and containers
  without copies.
- Function scope or file scope aliases of Eigen types (e.g.
  `using Vec3 = Eigen::Vector3<double>; using Arr3 = Eigen::Array<double, 3>`)
  are prohibited. These aliases almost always introduce confusion with
  ROS/GLM/etc. Vector types. Rely on modern Eigen's builtin aliases, e.g.,
  `Eigen::Vector3<...>` for brevity.
- Class scope aliases of Eigen types are allowed if they tie arguments/return
  values to the class's own template parameters.
- `auto` is encouraged, except when binding an Eigen expression.
- Functions templates should allow their template parameters to be deduced from
  their arguments, i.e. avoid explicit template parameters unless they are
  non-deducible.
- Prefer stateless objects holding only a `<Name>Cfg` struct; methods pure and
  `const`.
- "Bags of data" stay as aggregates — no positional constructors unless they
  validate. Construct with designated initialization.
- Concepts on Eigen args check dimensions, not binary compatibility.
- Spatial rotations and transforms obey the naming scheme `(tform|rot)_from2to`
  where `tform` and `rot` denote SE(3) and SO(3) (Euclidean transform vs spatial
  rotation) respectively, and `from` denotes the quantity/reference frame before
  transformation and `to` that after transformation. In mathematical notation,
  since the matrix forms of transformations are left-multiplied to a vector, the
  subscripts put `to` to the left of `from`, i.e. `\mathbf{R}_{ib}` is a
  rotation **from** body **to** world.

## Commit messages

- 5 lines maximum (subject + blank + ≤3 body lines).
- Long-form narrative goes in CHANGELOG.md, not the commit body.
- Each commit that updates a CHANGELOG entry references it by section:
  `See CHANGELOG entry 23.` or similar.
