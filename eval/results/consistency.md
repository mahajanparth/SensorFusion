Full run. "in band" is the share of samples inside the per-sample 95% chi-square region (ideal 95%). "(over)" means the mean is above the 95% bound for the mean, i.e. the filter is overconfident; "(under)" means underconfident.

### NIS (innovation vs S)

| variant | mean | dof | 95% bound on mean | in band |
|---|---:|---:|---:|---:|
| A | 34.4 (over) | 7 | 6.78-7.22 | 30 % |
| B | 173 (over) | 9 | 8.75-9.25 | 11 % |
| C | 0.919 (under) | 6 | 5.80-6.21 | 16 % |
| D | 0.226 (under) | 2 | 1.88-2.12 | 29 % |

### Position NEES vs GPS

| variant | mean | dof | 95% bound on mean | in band |
|---|---:|---:|---:|---:|
| A | 407 (over) | 2 | 1.75-2.26 | 16 % |
| B | 361 (over) | 2 | 1.75-2.26 | 17 % |
| C | 0.298 (under) | 2 | 1.75-2.26 | 61 % |
| D | 2.2 | 2 | 1.75-2.26 | 82 % |

### Heading NEES vs /odom reference

| variant | mean | dof | 95% bound on mean | in band |
|---|---:|---:|---:|---:|
| A | 4.76 (over) | 1 | 0.92-1.09 | 61 % |
| B | 12.3 (over) | 1 | 0.92-1.09 | 44 % |
| C | 1.4 (over) | 1 | 0.92-1.09 | 86 % |
| D | 1.15 (over) | 1 | 0.92-1.09 | 99 % |

