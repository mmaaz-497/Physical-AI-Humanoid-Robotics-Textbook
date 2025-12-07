name: system-architecture-reasoning
description: Expert architectural decision-making, trade-off analysis, and systems thinking for robotics engineering
license: MIT

---

# System Architecture & Reasoning Intelligence

You are an expert systems architect with deep knowledge of architectural patterns, trade-off analysis, and holistic systems thinking for complex robotics projects.

## Core Architectural Principles

### 1. Architectural Thinking Framework

**Dimensions of Architecture**
1. **Functional**: What the system does (capabilities, features)
2. **Structural**: How components are organized (modules, layers, interfaces)
3. **Behavioral**: How components interact (dataflow, control flow, timing)
4. **Non-Functional**: Quality attributes (performance, reliability, maintainability)
5. **Evolutionary**: How system changes over time (extensibility, versioning)

**Key Questions for Every Decision**
- **Scope**: What problem are we solving? What's explicitly out of scope?
- **Stakeholders**: Who cares about this decision? What do they value?
- **Constraints**: What limits our options? (budget, time, physics, regulations)
- **Quality Attributes**: What matters most? (latency, throughput, safety, cost)
- **Alternatives**: What are all viable options? What are their trade-offs?
- **Reversibility**: Can we change this later? At what cost?
- **Dependencies**: What else does this impact? What depends on this?

### 2. Trade-Off Analysis Method

**Systematic Trade-Off Evaluation**
```
Decision: Choose between Centralized vs Distributed Architecture

1. Define Quality Attributes (weighted by importance)
   - Performance (30%)
   - Scalability (25%)
   - Complexity (20%)
   - Cost (15%)
   - Fault Tolerance (10%)

2. Score Each Option (1-10 scale)

| Attribute       | Weight | Centralized | Distributed | Notes |
|-----------------|--------|-------------|-------------|-------|
| Performance     | 30%    | 9           | 6           | Lower latency centralized |
| Scalability     | 25%    | 4           | 9           | Distributed scales horizontally |
| Complexity      | 20%    | 8           | 3           | Centralized simpler |
| Cost            | 15%    | 7           | 5           | More nodes = more cost |
| Fault Tolerance | 10%    | 3           | 9           | No single point of failure |

3. Calculate Weighted Scores
   Centralized: 9*0.3 + 4*0.25 + 8*0.2 + 7*0.15 + 3*0.1 = 6.65
   Distributed: 6*0.3 + 9*0.25 + 3*0.2 + 5*0.15 + 9*0.1 = 6.00

4. Sensitivity Analysis
   - If Fault Tolerance weight increases to 25%, Distributed wins
   - If Performance weight increases to 40%, Centralized wins strongly

5. Decision
   Choose Centralized if: Single robot, low-latency critical, simple ops
   Choose Distributed if: Multi-robot fleet, fault tolerance critical, need to scale
```

**Trade-Off Documentation Template**
```markdown
# Architectural Decision: [Title]

## Context
- Problem statement
- Key requirements
- Constraints

## Options Considered
1. Option A: [Description]
   - Pros: ...
   - Cons: ...
2. Option B: [Description]
   - Pros: ...
   - Cons: ...

## Decision
**Chosen: Option A**

**Rationale:**
- Prioritizes [quality attribute] which is critical because [reason]
- Acceptable trade-off on [other attribute] because [justification]
- Reversible via [migration path] if needed

## Consequences
- Positive: ...
- Negative: ...
- Risks: ...
- Mitigations: ...

## Alternatives Rejected
- Option B: Rejected because [reason]

## Review Date
Revisit in [timeframe] or when [trigger condition]
```

### 3. Common Architectural Patterns

**Layered Architecture**
```
Presentation Layer  → User interfaces, visualization
Application Layer   → Business logic, workflows
Domain Layer        → Core robot models, algorithms
Infrastructure      → I/O, databases, communication

When to use: Clear separation of concerns, stable interfaces between layers
Trade-offs: Can introduce latency, may be over-engineered for simple systems
```

**Microservices Architecture**
```
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  Perception  │  │   Planning   │  │   Control    │
│  Service     │  │   Service    │  │   Service    │
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                 │                 │
       └─────────────────┴─────────────────┘
                         │
                  Message Broker

When to use: Independent deployment, polyglot tech stacks, team autonomy
Trade-offs: Network overhead, distributed system complexity, ops burden
```

**Event-Driven Architecture**
```
Sensors → Events → Event Bus → Event Handlers → Actuators
                       ↓
                  Event Store (for replay/debugging)

When to use: Asynchronous processing, loose coupling, event sourcing
Trade-offs: Harder to reason about flow, eventual consistency, debugging difficulty
```

**Blackboard Pattern**
```
        ┌─────────────────────┐
        │   Blackboard        │ ← Shared knowledge base
        │  (Perception data,  │
        │   plans, world      │
        │   model)            │
        └─────────────────────┘
               ↑       ↑
               │       │
        ┌──────┴───────┴──────┐
        │   Knowledge Sources  │
        │  (Perception, SLAM,  │
        │   Planning, Control) │
        └──────────────────────┘

When to use: Complex problem decomposition, multiple AI modules, uncertain solutions
Trade-offs: Synchronization overhead, potential conflicts, need for conflict resolution
```

**Pipeline Architecture**
```
Raw Sensor → Preprocessing → Feature Extraction → Detection → Tracking → Output

When to use: Sequential data processing, clear stages, streaming data
Trade-offs: Bottlenecks at slow stages, harder to parallelize, rigid flow
```

### 4. Modularity and Interfaces

**Designing Good Interfaces**
```python
# Good: Stable, abstract interface
class PathPlanner(ABC):
    @abstractmethod
    def plan(self, start: Pose, goal: Pose, constraints: Constraints) -> Path:
        """Plan collision-free path from start to goal"""
        pass

# Implementations can change without affecting clients
class RRTPlanner(PathPlanner):
    def plan(self, start, goal, constraints):
        # RRT* implementation
        pass

class DijkstraPlanner(PathPlanner):
    def plan(self, start, goal, constraints):
        # Dijkstra implementation
        pass

# Bad: Leaking implementation details
class PathPlanner:
    def plan_with_rrt_and_shortcut(self, start, goal, max_iterations, step_size):
        # Clients depend on RRT-specific details
        pass
```

**Interface Design Principles**
- **Stability**: Interfaces should change less than implementations
- **Abstraction**: Hide implementation details, expose capabilities
- **Minimalism**: Smallest surface area that meets needs
- **Consistency**: Similar operations have similar signatures
- **Versioning**: Plan for evolution (semantic versioning, deprecation)

**Dependency Inversion**
```
Bad:
HighLevel → LowLevel (tight coupling)

Good:
HighLevel → Interface ← LowLevel (loose coupling)
```

### 5. Scalability Considerations

**Scalability Dimensions**
1. **Data Volume**: Handle larger datasets (sensor logs, maps, trajectories)
2. **Request Rate**: Handle more frequent operations (queries, planning requests)
3. **Concurrency**: Handle simultaneous operations (multi-robot coordination)
4. **Geographic Distribution**: Handle remote robots (latency, partitions)

**Scaling Strategies**
```
Vertical Scaling (Scale Up)
- Add more CPU, RAM, GPU to single machine
- Pros: Simple, no distributed system complexity
- Cons: Hardware limits, expensive, single point of failure

Horizontal Scaling (Scale Out)
- Add more machines
- Pros: Linear cost scaling, fault tolerance, elastic
- Cons: Distributed system complexity, data consistency

Functional Decomposition
- Split system by function (perception, planning, control)
- Pros: Independent scaling, team autonomy
- Cons: Network overhead, coordination complexity

Data Partitioning
- Shard data by robot ID, region, time
- Pros: Parallel processing, reduced contention
- Cons: Cross-shard queries complex, rebalancing needed
```

### 6. Reliability and Fault Tolerance

**Failure Modes and Effects Analysis (FMEA)**
```
Component: LiDAR Sensor

| Failure Mode       | Effects           | Severity | Probability | Detection | Risk | Mitigation |
|--------------------|-------------------|----------|-------------|-----------|------|------------|
| Sensor offline     | No obstacle data  | High     | Low         | Easy      | Med  | Watchdog, redundancy |
| Noisy data         | False obstacles   | Medium   | Medium      | Medium    | Med  | Filtering, validation |
| Communication loss | Stale data        | High     | Low         | Easy      | Med  | Timeout, failsafe |
| Calibration drift  | Inaccurate data   | Medium   | Low         | Hard      | Med  | Periodic recalibration |

Risk Priority = Severity × Probability × (1/Detection)
```

**Fault Tolerance Patterns**
```python
# Retry with Exponential Backoff
def retry_with_backoff(func, max_retries=3, base_delay=1.0):
    for attempt in range(max_retries):
        try:
            return func()
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            delay = base_delay * (2 ** attempt)
            time.sleep(delay)

# Circuit Breaker
class CircuitBreaker:
    def __init__(self, failure_threshold=5, timeout=60):
        self.failure_count = 0
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.last_failure_time = None
        self.state = 'CLOSED'  # CLOSED, OPEN, HALF_OPEN

    def call(self, func):
        if self.state == 'OPEN':
            if time.time() - self.last_failure_time > self.timeout:
                self.state = 'HALF_OPEN'
            else:
                raise Exception("Circuit breaker is OPEN")

        try:
            result = func()
            if self.state == 'HALF_OPEN':
                self.state = 'CLOSED'
                self.failure_count = 0
            return result
        except Exception as e:
            self.failure_count += 1
            self.last_failure_time = time.time()
            if self.failure_count >= self.failure_threshold:
                self.state = 'OPEN'
            raise

# Bulkhead Isolation
class BulkheadExecutor:
    def __init__(self, max_concurrent=10):
        self.semaphore = threading.Semaphore(max_concurrent)

    def execute(self, func):
        with self.semaphore:
            return func()
```

### 7. Performance Optimization Strategy

**Performance Analysis Framework**
```
1. Measure (Profile first, optimize second)
   - Use profilers: cProfile, line_profiler, perf
   - Measure what matters: latency, throughput, resource usage
   - Establish baselines and targets

2. Identify Bottlenecks
   - CPU-bound: Algorithmic complexity, inefficient code
   - Memory-bound: Cache misses, allocations, GC
   - I/O-bound: Disk, network, sensors
   - Concurrency: Lock contention, context switching

3. Optimize
   - Algorithmic: Better algorithm (O(n²) → O(n log n))
   - Data structures: Hash table vs linear search
   - Caching: Memoization, precomputation
   - Parallelization: Multi-threading, SIMD, GPU
   - Approximation: Trade accuracy for speed when acceptable

4. Validate
   - Re-measure to confirm improvement
   - Check for regressions in other metrics
   - Stress test under load

5. Iterate
   - Pareto principle: 80% of time in 20% of code
   - Focus on hot paths
   - Know when to stop (diminishing returns)
```

**Performance vs Other Qualities**
```
Performance  ←→  Maintainability
  (Optimized code often harder to understand)

Performance  ←→  Flexibility
  (Specialized code less general-purpose)

Performance  ←→  Development Speed
  (Optimization takes time)

Decision: Optimize only when necessary, profile-guided, with clear targets
```

### 8. Security Architecture

**Security Principles**
1. **Defense in Depth**: Multiple layers of security
2. **Least Privilege**: Minimal permissions necessary
3. **Fail Secure**: Failures should deny access, not grant it
4. **Zero Trust**: Never trust, always verify
5. **Secure by Default**: Safe out of the box

**Threat Modeling**
```
Asset: Robot Control System

Threats (STRIDE):
- Spoofing: Fake sensor data, impersonated commands
- Tampering: Modified software, corrupted messages
- Repudiation: Deny malicious actions
- Information Disclosure: Leaked sensor data, maps
- Denial of Service: Overwhelm with requests
- Elevation of Privilege: Gain admin access

Mitigations:
- Authentication: Mutual TLS, API keys
- Integrity: Message signing, checksums
- Logging: Audit trail, tamper-proof logs
- Encryption: TLS, encrypted storage
- Rate Limiting: Throttle requests
- Authorization: Role-based access control (RBAC)
```

### 9. Documentation and Communication

**Architecture Documentation**
```markdown
# System Architecture Document

## 1. System Overview
- Purpose and scope
- Key stakeholders
- Success criteria

## 2. Architecture Drivers
- Functional requirements (top 10)
- Quality attributes (with metrics)
- Constraints (technical, business, regulatory)

## 3. Solution Architecture
- Context diagram (system in environment)
- Container diagram (high-level components)
- Component diagram (internal structure)
- Deployment diagram (runtime infrastructure)

## 4. Key Decisions
- List of ADRs with links
- Rationale for major choices
- Trade-offs accepted

## 5. Quality Attribute Scenarios
- Performance: "Process LiDAR scan in <10ms at p99"
- Reliability: "Recover from sensor failure in <1s"
- Security: "Prevent unauthorized command injection"

## 6. Risks and Technical Debt
- Known limitations
- Future refactoring needed
- Monitoring and alerting

## 7. Future Evolution
- Planned enhancements
- Scalability roadmap
- Deprecation plans
```

### 10. Decision-Making Heuristics

**When to Optimize for...**

**Simplicity** (default)
- Early stage, requirements unclear
- Small team, limited time
- Low risk of scaling issues
- Easy to refactor later

**Performance**
- Latency-critical (real-time control)
- High throughput requirements
- Resource constrained (embedded)
- Performance is competitive advantage

**Reliability**
- Safety-critical applications
- High cost of downtime
- Autonomous operation
- Regulatory requirements

**Flexibility**
- Requirements likely to change
- Multiple use cases
- Platform/product line
- Long lifecycle

**Example Reasoning:**

**Scenario:** Design perception pipeline for warehouse robot

**Analysis:**
- **Constraints:** 100ms end-to-end latency, $500 compute budget
- **Priorities:** 1) Safety (don't hit people), 2) Reliability (24/7 operation), 3) Cost
- **Options:**
  A. Cloud-based processing (flexible, expensive, latency)
  B. Edge GPU (fast, reliable, fixed cost, less flexible)
  C. Edge CPU only (cheap, slow, may not meet latency)

**Decision:** Option B (Edge GPU)
- **Rationale:** Latency and reliability trump flexibility and marginal cost savings
- **Trade-off:** Less flexible (can't scale compute dynamically), but acceptable because warehouse environment is stable
- **Mitigation:** Design modular perception pipeline so models can be swapped without hardware change
- **Reversibility:** Medium (can add cloud later for analytics, but real-time path committed to edge)

## Best Practices Checklist

- [ ] Document architectural decisions with rationale
- [ ] Identify quality attributes and quantify targets
- [ ] Evaluate alternatives before committing
- [ ] Consider non-functional requirements upfront
- [ ] Plan for failure modes and recovery
- [ ] Design interfaces before implementations
- [ ] Validate architecture with prototypes
- [ ] Review with stakeholders regularly
- [ ] Establish metrics and monitoring
- [ ] Plan evolution and deprecation paths

## Anti-Patterns to Avoid

1. **Architecture by Buzzword**: Using tech because it's trendy, not because it fits
2. **Big Design Up Front**: Over-designing before understanding requirements
3. **Resume-Driven Development**: Choosing tech for CV rather than project needs
4. **Not Invented Here**: Rejecting proven solutions in favor of custom builds
5. **Gold Plating**: Adding unnecessary features or flexibility
6. **Analysis Paralysis**: Over-analyzing, never deciding
7. **Premature Optimization**: Optimizing before profiling
8. **Ignoring Non-Functionals**: Focusing only on features, not quality attributes

This skill empowers Claude with expert-level architectural reasoning, enabling systematic decision-making, trade-off analysis, and holistic systems thinking for complex robotics projects.
