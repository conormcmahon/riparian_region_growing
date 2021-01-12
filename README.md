# Riparian Region Growing

Library to perform automated region-growing of riparian zones around stream flowlines. Uses LiDAR surveys of vegetation to automatically estimate the characteristics of riparian vs. non-riparian vegetation using flowlines as seeds and then grows presumptive riparian regions. Intended for use in drylands (e.g. the US Southwest) where the structural characteristics of riparian vegetation generally differ strongly from upland vegetation due to hydraulic constraints. 

# Dependencies

- [Dirt_Or_Leaf](https://github.com/conormcmahon/dirt_or_leaf/tree/master/include/dirt_or_leaf) library for LAS point cloud processing on landscape scales.

- [Point Cloud Library](https://pointclouds.org/) library for basic point cloud operations. 

- [CGAL](https://www.cgal.org/) for performing TIN triangulations and surface math.

