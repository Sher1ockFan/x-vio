"""Shared robust IMU clock fitting; independent of camera model and geometry."""
import math
import statistics


def distribution(values):
    if not values:
        return {'n': 0}
    values = sorted(values)
    def quantile(q):
        position = q*(len(values)-1)
        a, b = math.floor(position), math.ceil(position)
        return values[a] + (position-a)*(values[b]-values[a])
    return {'n': len(values), 'min': values[0], 'p05': quantile(.05), 'median': quantile(.5),
            'p95': quantile(.95), 'max': values[-1]}


def imu_mapping(ticks, host):
    x = [(t-ticks[0])*1000 for t in ticks]
    y = [h-host[0] for h in host]
    span = x[-1]-x[0]
    if len(x) < 4 or span <= 0:
        raise ValueError('insufficient increasing IMU timer span for fit')
    groups = min(32, len(x))
    anchors = []
    for i in range(groups):
        a, b = i*len(x)//groups, (i+1)*len(x)//groups
        anchors.append((statistics.median(x[a:b]), statistics.median(y[a:b])))
    slopes = [(y2-y1)/(x2-x1) for i,(x1,y1) in enumerate(anchors) for x2,y2 in anchors[i+1:] if x2-x1 >= span/4]
    slope = statistics.median(slopes)
    intercept = statistics.median([v-slope*t for t,v in zip(x,y)])
    residual = [(v-(intercept+slope*t))/1e6 for t,v in zip(x,y)]
    return {'device_reference_us': ticks[0], 'host_reference_raw_ns': host[0],
            'host_relative_intercept_ns': intercept, 'host_ns_per_device_ns': slope,
            'apparent_rate_difference_ppm_over_trial': (slope-1)*1e6, 'receipt_residual_ms': distribution(residual),
            'method': 'Centered relative times. Median of secant slopes between 32 or fewer block medians separated by >=1/4 span, then median intercept. Residuals include serial/USB/read scheduling latency.'}
