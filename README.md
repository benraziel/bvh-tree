bvh-tree
======

A Bounding Volume Hierarchy data structure written in javascript, for spatial indexing of large triangle meshes.
Enables fast intersection of rays with a triangle mesh.


## Demo 
[ray-mesh intersection](https://cdn.rawgit.com/benraziel/bvh-js/master/example/index.html)

## Usage

### Construct a BVH from a list of triangles
```js
var triangle0 = [
  {x: 0.0, y: 0.0, z: 0.0}, 
  {x: 0.0, y: 0.0, z: 100.0}, 
  {x: 0.0, y: 100.0, z: 100.0}
];

var triangle1 = [
  {x: 0.0, y: 0.0, z: 0.0}, 
  {x: 0.0, y: 0.0, z: -100.0}, 
  {x: 0.0, y: -100.0, z: -100.0}
];

// the maximum number of triangles that can fit in a node before splitting it.
var maxTrianglesPerNode = 7; 

var bvh = new BVH([triangle0, triangle1], maxTrianglesPerNode);
```

### Intersect a ray with the BVH
```js

// origin point of the ray
var rayOrigin = {x: 0.0, y: 0.0, z: 0.0};

// direction of the ray. should be normalized to unit length
var rayDirection = {x: 0.0, y:0.0, z:-1.0};

// if 'true', only intersections with front-faces of the mesh will be performed
var backfaceCulling = true;

var intersectionResult = bvh.intersectRay(rayOrigin, rayDirection, backfaceCulling);
```
