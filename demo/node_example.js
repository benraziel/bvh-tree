var bvhtree = require('../bvhtree.js');

var triangle0 = [{x: 0.0, y: 0.0, z: 0.0}, {x: 1000.0, y: 0.0, z: 0.0}, {x:1000.0, y:1000.0, z:0.0}];
var triangle1 = [{x: 0.0, y: 0.0, z: 0.0}, {x: 2000.0, y: 0.0, z: 0.0}, {x:2000.0, y:1000.0, z:0.0}];

// the maximum number of triangles that can fit in a node before splitting it.
var maxTrianglesPerNode = 7;
var bvh = new bvhtree.BVH([triangle0, triangle1], maxTrianglesPerNode);

// origin point of the ray
var rayOrigin = {x: 1500.0, y: 3.0, z:1000};

// direction of the ray. should be normalized to unit length
var rayDirection = {x: 0, y:0, z:-1};

// if 'true', only intersections with front-faces of the mesh will be performed
var backfaceCulling = true;
var intersectionResult = bvh.intersectRay(rayOrigin, rayDirection, backfaceCulling);

console.log(intersectionResult);