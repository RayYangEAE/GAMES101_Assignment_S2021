# 作业6：加速结构

1. 包围盒求交。
2. BVH查找。
3. SAH查找：SAH是基于表面积的启发式评估划分方法。BVH划分包围盒会有重叠部分，SAH计算cost使包围盒重叠部分尽量小（尽管可能造成树的深度加深），使得包围盒剔除的效率更好，减少三角形求交计算，从而减少渲染时的计算量。cost=(surfaceArea1 * cnt1 + surfaceArea2 * cnt2) / surfaceArea + cost_trav.
