## Radar final project
The selected range of the target is 120m and velocity of the target is 10.

### 2D CFAR 
The calculation of the training signal was done following the description in the provided code. Having 2 loops iterating over rows and columns of RDM matrix. Taking into account the number of training and guarding cells, I sum all values first and then substract what is stored in guard cells only in order to keep only the values from the training cells. Afterwards, adding an offset to the training cell values we get our threshold. Checking if the current cell is above or below threshold, we binarise the RDM_copy matrix to store 1 or 0 respectively. This makes the peak of the signal to show as maximum value in the figure.
### Selection of hyper parameters
The selected values for the 2D CFAR are as following:
* Td = 8;
* Gd = 4;
* Tr = 10;
* Gr = 4;
* offset = 7;

The selected offset removes the most of the noise and repeated signals that would otherwise pass through the threshold. The guarding cell number was first selected 2 but settled on 4 as this way I was not recording non peak signals that are above threshold. The value for **Td** and **Tr** took more experimentation as this impacted the performance of range and velocity detection. Increasing or decreasing this value would move it further away from actual target distance/speed or closer and lower speed compared to actual. The settled values are 10 for **Tr** and 8 for **Td** as this provided the closest results to the actual value.
### Removing boundaries
To remove the values at the boundaries and they are limited by hyper paramater values, I set to 0 all values that are in all rows within 1:Tr+Gr and last Tr+Gr rows when checking from the middle of the ranges and fft in range is double sided and we want only one. Same goes for columns, making all rows that were not set to 0 with the first zeroing.