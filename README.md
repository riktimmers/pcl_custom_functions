# pcl_custom_functions
Some custom implementations of the Point Cloud Library functions that run faster then the default PCL implementation.

Tested with PCL-1.10 under Ubuntu 20.04.
```
$ sudo apt install libpcl-dev
```

To build the code: 
```
$ mkdir -p build/Release
$ cd build/Release
$ cmake -DCMAKE_BUILD_TYPE=Release ../../
$ make -j4
```

To run the main program: 
```
$ ./main
```
To run the program with timings:
```
$ ./timing_main
```

Results of timing on 3 different platforms:
```
Jetson Nano run:
(via docker running pcl-1.10)

Passthrough: 25.765 milliseconds
Segmentation: 43.236 milliseconds
Clustering: 1723.8 milliseconds
Pcl total: 1800.63 milliseconds
Clusters: 1
Custom passthrough: 5.643 milliseconds
Custom segmentation: 36.633 milliseconds
Custom clustering: 15.099 milliseconds
Pcl custom total: 67.439 milliseconds
Clusters: 1

PC (AMD Ryzen 7 3700X) run:
Passthrough: 6.889 milliseconds
Segmentation: 10.891 milliseconds
Clustering: 552.858 milliseconds
Pcl total: 574.487 milliseconds
Clusters: 1
Custom passthrough: 1.79 milliseconds
Custom segmentation: 4.201 milliseconds
Custom clustering: 3.999 milliseconds
Pcl custom total: 13.026 milliseconds
Clusters: 1

Laptop (AMD Ryzen 5 4500u) run:
Passthrough: 13.532 milliseconds
Segmentation: 10.579 milliseconds
Clustering: 508.383 milliseconds
Pcl total: 536.666 milliseconds
Clusters: 1
Custom passthrough: 2.168 milliseconds
Custom segmentation: 3.733 milliseconds
Custom clustering: 3.92 milliseconds
Pcl custom total: 13.294 milliseconds
Clusters: 1
```


