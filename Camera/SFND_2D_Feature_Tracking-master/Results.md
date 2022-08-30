## Results of all descriptor and detector combinations

From the spreadsheet (comparison_results.csv file) in the Results folder these observations can be made:

* FAST detector is by far the best of all used;

* BRIEF and ORB descriptors performed the best regardless of the detector;

* BRISK, SIFT and AKAZE detectors find the most keypoints which can be a result for their performance;

Adjusted parameters could alter these results in one or other favour, however, with the default parameters
these were the results:

### N of keypoints detected on the vehicle in descending order

* BRISK = 2762;
* AKAZE = 1670;
* SIFT = 1386;
* SHITOMASI = 1179;
* ORB = 1161;
* FAST = 564;
* HARRIS = 248;

### N of matched keypoints using BF and KNN with k = 2 and distance ratio threshold of 0.8

| Detector/Descriptor | AKAZE | BRIEF | BRISK | FREAK | ORB | SIFT |
|---------------------|-------|-------|-------|-------|-----|------|
| AKAZE               | 697   | 708   | 683   | 659   | 673 | 699  |
| BRISK               | N/A   | 936   | 859   | 813   | 814 | 899  |
| FAST                | N/A   | 233   | 201   | 196   | 228 | 213  |
| HARRIS              | N/A   | 92    | 78    | 77    | 87  | 87   |
| ORB                 | N/A   | 281   | 412   | 238   | 413 | 416  |
| SHITOMASI           | N/A   | 521   | 424   | 425   | 502 | 508  |
| SIFT                | N/A   | 395   | 329   | 335   | N/A | 443  |

### Time performance of different detector+descriptor pairs in ms with BF and KNN (k=2 and ratio threshold set to 0.8)

| Detector/Descriptor | AKAZE   | BRIEF   | BRISK   | FREAK   | ORB     | SIFT    |
|---------------------|---------|---------|---------|---------|---------|---------|
| AKAZE               | 848.134 | 466.566 | 768.077 | 662.931 | 478.849 | 609.158 |
| BRISK               | N/A     | 646.183 | 963.716 | 855.53  | 669.693 | 812.542 |
| FAST                | N/A     | 6.314   | 314.267 | 206.093 | 10.332  | 106.003 |
| HARRIS              | N/A     | 101.47  | 410.502 | 300.97  | 105.196 | 194.97  |
| ORB                 | N/A     | 57.97   | 367.575 | 255.301 | 87.8392 | 262.472 |
| SHITOMASI           | N/A     | 116.436 | 447.495 | 273.903 | 116.103 | 170.444 |
| SIFT                | N/A     | 812.225 | 1002.74 | 1022.74 | N/A     | 1243.4  |

## Conclusion

The FAST detector works best with BRIEF, ORB and SIFT descriptors and producing relatively large number of keypoints. On the other hand, even being dated, SHITOMASI detector is not the worst compared to others, where the worst performing was SIFT combined with any of the descriptors. During the project work, it was noticed that SIFT detector and ORB descriptor do not work together (you will get out of memory error). One point that stood out is that though ORB has similar methodology to FAST+BRIEF, the performance is lower (time wise). The two detectors worth mentioning for finalists are FAST and ORB - each identified similar keypoints, where ORB had fewer outliers to far distant vehicles in the background. Nevertheless, both of them identified keypoints mostly on the vehicle in front. Therefore, the 3 selected combinations of detector+descriptor are:

* FAST + BRIEF;
* FAST + ORB;
* ORB + BRIEF;
