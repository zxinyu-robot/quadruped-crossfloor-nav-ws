# Architecture

## Scope

Fork Spatial Runtime owns the consistency boundary between measurement-to-factor adapters and an optimizer backend. It deliberately does not own sensor drivers, residual mathematics, nonlinear optimization, mapping, planning, or robot control.

## Core invariants

1. A state key is unique within a runtime instance.
2. A factor ID is accepted at most once.
3. A factor can reference only active or fixed states.
4. Covariance entries accepted by the current model are finite and positive.
5. A graph version advances only after the backend accepts the full transaction.
6. Marginalization first computes a prior and then commits lifecycle transitions and that prior as one transaction.

## Submission path

`SpatialRuntime::submit()` creates candidate copies of the state index and factor-ID set. State requests and factors are validated against those candidates, allowing one batch to create states and immediately reference them. The backend receives a `GraphTransaction` only after all validation succeeds.

If the backend rejects the transaction, candidate data is discarded and the public graph version is unchanged.

## Window advancement

An `IWindowPolicy` selects states from an immutable record view. `FixedLagWindowPolicy` selects active temporal states older than the watermark minus the configured lag while preserving sensor extrinsics and clock offsets.

The optimizer backend computes a marginal prior without mutating itself. The runtime then submits state transitions and the optional prior through the same atomic `apply()` boundary.

## Concurrency model

The current prototype serializes submission, window advancement, optimization, and snapshots with one mutex. This favors explicit consistency over throughput. Future lock splitting must preserve graph-version and state-lifecycle invariants and should be justified by benchmark data.

## Integration boundary

A production backend should:

- map generic state and factor types to concrete manifold values and residuals;
- apply each transaction atomically;
- return a snapshot associated with the requested graph version;
- compute marginal priors without changing live backend state.

ROS 2, DDS, GTSAM, and Ceres integrations belong in adapters rather than the runtime core.
