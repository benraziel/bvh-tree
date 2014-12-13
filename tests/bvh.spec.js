describe("bvhTests", function() {
    var twoTrianglesX = [
        -5.0, 20.0, -30.0, -2.0, -50.0, 60.0, -1.0, 80.0, -90.0,
        5.0, 21.0, -31.0, 2.0, -51.0, 61.0, 1.0, 81.0, -91.0
    ];

    var threeTriangles = [
        -10.0, 20.0, -30.0, 40.0, -50.0, 60.0, -70.0, 80.0, -90.0,
        -11.0, 21.0, -31.0, 41.0, -51.0, 61.0, -71.0, 81.0, -91.0,
        -12.0, 22.0, -32.0, 42.0, -55.0, 62.0, -78.0, 87.0, -92.0
    ];

    it("Should allow constructing an empty tree", function () {
        var bvh = new BVH([], 10);
        expect(bvh._trianglesArray).toEqual([]);
        expect(bvh._rootNode._extentsMin).toEqual({'x': 0, 'y': 0, 'z': 0});
        expect(bvh._rootNode._extentsMax).toEqual({'x': 0, 'y': 0, 'z': 0});
    });

    it("Should correctly calculate node extents box for a single triangle", function () {
        var triangle = [-1.0, 2.0, -3.0, 4.0, -5.0, 6.0, -7.0, 8.0, -9.0];

        var bvh = new BVH(triangle, 10);
        expect(bvh._rootNode._extentsMin).toEqual({'x': -7.0, 'y': -5.0, 'z': -9.0});
        expect(bvh._rootNode._extentsMax).toEqual({'x': 4.0, 'y': 8.0, 'z': 6.0});
    });

    it("Should correctly calculate node extents for multiple triangles", function () {
        var bvh = new BVH(twoTrianglesX, 10);
        expect(bvh._rootNode._extentsMin).toEqual({'x': -5.0, 'y': -51.0, 'z': -91.0});
        expect(bvh._rootNode._extentsMax).toEqual({'x': 5.0, 'y': 81.0, 'z': 61.0});
    });

    it("Should not split a node before it has reached the maximum number of triangles", function () {
        var bvh = new BVH(threeTriangles, 3);
        expect(bvh._rootNode._node0).toBeNull();
        expect(bvh._rootNode._node1).toBeNull();
    });

    it("Should split a node when the maximal number of triangles has been reached", function () {
        var bvh = new BVH(threeTriangles, 2);
        expect(bvh._rootNode._node0 instanceof BVHNode).toBeTruthy();
        expect(bvh._rootNode._node1 instanceof BVHNode).toBeTruthy();
    });

    it("Should split according to the right axis, and create new nodes with correct data", function() {
        var bvh = new BVH(twoTrianglesX, 1);

        // check that the node has been split by the X axis (in our case, x = 0)
        expect(bvh._rootNode.elementCount()).toEqual(0);
        expect(bvh._rootNode._node0.elementCount()).toEqual(1);
        expect(bvh._rootNode._node1.elementCount()).toEqual(1);

        // check that correct bounding boxes has been calculated
        expect(bvh._rootNode._node0._extentsMin.x).toEqual(-5.0);
        expect(bvh._rootNode._node0._extentsMax.x).toEqual(-1.0);
        expect(bvh._rootNode._node1._extentsMin.x).toEqual(1.0);
        expect(bvh._rootNode._node1._extentsMax.x).toEqual(5.0);
    });
});