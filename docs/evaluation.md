# Evaluation plan

## Perception

Record field, date, camera setup, crop species, weed species, growth stage, and lighting for each capture. Annotate plant instances and class labels, including weeds within crop rows and overlapping plants. Keep a session-level train/validation/test split. Use validation data to choose segmentation thresholds and crop bands; reserve test data for the final comparison.

The CLI evaluates classification on extracted plant proposals. Report confusion counts as well as precision, recall, and accuracy, including sample counts and class balance. For end-to-end detection, separately match detections to ground-truth instances with a fixed IoU or centroid-distance threshold and one-to-one matching. Include missed plants, merged contours, duplicate detections, and weeds suppressed by crop protection. The current CLI does not implement this instance-level evaluator.

Compare the row baseline against the learned classifier on the same images and record failure examples. Measure processing latency on a named machine using repeated runs; report median and high-percentile latency. Synthetic shapes are useful for regression tests but cannot establish field accuracy.

## Robot targeting

Measure projection error at independent ground points, followed by tool positioning error at reachable targets. Report errors in millimetres, target count, workspace coverage, and calibration setup. Separate camera error, kinematic error, and mechanical repeatability. Do not equate the numerical IK convergence threshold with physical accuracy.

Measure task completion, rejected targets, fault recovery, crop contacts, and cycle time. For the paper's uprooting task, distinguish successful grasp, complete root extraction, successful basket deposit, and crop damage. Test across soil moisture, root sizes, and plant spacing. A simulated sequence completing does not establish any of these outcomes. The current simulator has no time dynamics, so it cannot measure scheduling speedups. Compare an FSM-driven physical trial against a defined baseline with the same targets, hardware, and task endpoint.

## Agricultural outcomes

The CV's reported 30% precision gain, 20% labor-cost reduction, 40% efficiency gain, 95% detection accuracy, and 25% herbicide reduction require independent evidence. The supplied paper does not validate these percentages. Its literature-review detection rates describe other projects and must not be reused as Krishak results.

Define each measurement before trials: what counts as a precise agricultural action, which labor activities and rates are counted, whether efficiency means targets per minute or another quantity, and how detection accuracy is calculated. The paper's method is herbicide-free mechanical removal. To assess reduced chemical use, compare measured treatment needs and weed-control outcomes against a clearly defined baseline over time. A robot having no sprayer does not by itself establish a percentage reduction in farm herbicide consumption. There is no dispensing subsystem in this repository.

Store raw observations, baseline results, exclusions, sample sizes, and uncertainty alongside any summary. Until those records exist, retain the values as source-reported claims.
