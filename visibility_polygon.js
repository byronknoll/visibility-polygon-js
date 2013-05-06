/*
This code is released into the public domain - attribution is appreciated but not required.
Made by Byron Knoll in 2013.

https://code.google.com/p/visibility-polygon-js/
Demo: http://www.byronknoll.com/visibility.html

This library can be used to construct a visibility polygon for a set of obstacles (polygons).

The time complexity of this implementation is O(N^2) (where N is the total number of polygon vertices). There exist algorithms with a better time complexity (but they are significantly more complicated to implement). As far as I know there does not exist a O(N) algorithm for this problem. There exists a O(N) algorithm for the simpler problem of computing a visibility polygon for a *single* enclosing polygon.

The following three functions should be useful:

1) VisibilityPolygon.compute(position, polygons)
  Computes a visibility polygon. O(N^2) time complexity.
  Arguments:
    position - The location of the observer. Must not be within an obstacle (see the inObstacle function below).
    polygons - A list of polygons representing obstacles. The first polygon should be the outer boundary - all other polygons should be contained within the boundary and non-intersecting. All polygons should be specified in clockwise vertex order (see the convertToClockwise function below).
  Returns: The visibility polygon (in clockwise vertex order).

2) VisibilityPolygon.inObstacle(position, polygons)
  Calculates whether a point is within an obstacle. O(N) time complexity.
  Arguments: same as compute function above.
  Returns: True if "position" is within an obstacle.

3) VisibilityPolygon.convertToClockwise(polygons)
  Converts the given polygons to clockwise vertex order. O(N) time complexity.
  Arguments: a list of polygons (in either clockwise or counterclockwise vertex order).
  Returns: a list of polygons in clockwise vertex order.

Example code:

var polygons = [];
polygons.push([[-1,-1],[501,-1],[501,501],[-1,501]]);
polygons.push([[250,100],[260,140],[240,140]]);
polygons = VisibilityPolygon.convertToClockwise(polygons);
var position = [10, 10];
if (!VisibilityPolygon.inObstacle(position, polygons)) {
  var visibility = VisibilityPolygon.compute(position, polygons);
}

*/

function VisibilityPolygon(){};

VisibilityPolygon.compute = function(position, polygons) {
	var sorted = VisibilityPolygon.sortPoints(position, polygons);
	var map = VisibilityPolygon.makeMap(polygons);
	var polygon = [];
	var vertex = VisibilityPolygon.firstVisibleVertex(position, polygons);
	var start = [vertex[0][0], vertex[0][1]];
	var cur = vertex[0];
	var next = vertex[1];
	if (VisibilityPolygon.equal(cur, polygons[next[0]][next[1]])) {
		VisibilityPolygon.increment(next, polygons[next[0]].length);
	}
	var first = true;
	for (var i = 0; i < 2 * sorted.length; ++i) {
		polygon.push(cur);
		VisibilityPolygon.hop(cur, next, map, polygons, position);
		var end = VisibilityPolygon.closestPoint(cur, polygons[next[0]][next[1]], start);
		if (VisibilityPolygon.equal(end, start) && !first) break;
		var a = VisibilityPolygon.angle2(polygons[next[0]][next[1]], cur, position);
		if (a <= 0 || a >= 180) {
			cur = VisibilityPolygon.extend(cur, next, polygons, position);
		} else {
			cur = VisibilityPolygon.trace(cur, next, sorted, polygons, position, polygon);
		}
		first = false;
	}
	return polygon;
};

VisibilityPolygon.inObstacle = function(position, polygons) {
	var edge = polygons[0][0];
	var parity = 0;
	for (var i = 1; i < polygons.length; ++i) {
		for (var j = 0; j < polygons[i].length; ++j) {
			var k = j + 1;
			if (k == polygons[i].length) k = 0;
			if (VisibilityPolygon.doLineSegmentsIntersect(edge[0], edge[1], position[0], position[1], polygons[i][j][0], polygons[i][j][1], polygons[i][k][0], polygons[i][k][1])) {
				var intersect = VisibilityPolygon.intersectLines(edge, position, polygons[i][j], polygons[i][k]);
				if (VisibilityPolygon.equal(position, intersect)) return true;
				if (VisibilityPolygon.equal(intersect, polygons[i][j])) {
					if (VisibilityPolygon.angle2(position, edge, polygons[i][k]) < 180) ++parity;
				} else if (VisibilityPolygon.equal(intersect, polygons[i][k])) {
					if (VisibilityPolygon.angle2(position, edge, polygons[i][j]) < 180) ++parity;
				} else {
					++parity;
				}
			}
		}
	}
	return (parity%2)!=0;
};

VisibilityPolygon.convertToClockwise = function(polygons) {
	var polys = [];
	for (var i = 0; i < polygons.length; ++i) {
		if (VisibilityPolygon.isClockwise(polygons[i])) {
			polys.push(polygons[i]);
		} else {
			polys.push(new Array(polygons[i].length));
			var index = 0;
			for (var j = polygons[i].length - 1; j >= 0; --j) {
				polys[i][index] = new Array(2);
				polys[i][index][0] = polygons[i][j][0];
				polys[i][index][1] = polygons[i][j][1];
				++index;
			}
		}
	}
	return polys;
};

VisibilityPolygon.isClockwise = function(polygon) {
	var sum = 0;
	for (var i = 0; i < polygon.length; ++i) {
		var j = i + 1;
		if (j == polygon.length) j = 0;
		sum += ((polygon[j][0]-polygon[i][0]) * (polygon[j][1]+polygon[i][1]));
	}
	return sum < 0;
};

VisibilityPolygon.epsilon = function() {
	return 0.0000001;
}

VisibilityPolygon.hop = function(cur, next, map, polygons, position) {
	var key = cur[0] + "," + cur[1];
	if (key in map) {
		var best_a = -1;
		var a2 = VisibilityPolygon.angle2(polygons[next[0]][next[1]], cur, position);
		var arr = map[key];
		if (arr.length == 1) return;
		for (var i = 0; i < arr.length; ++i) {
			if (arr[i][1] == next[0]) continue;
			var k = arr[i][2];
			var j = k - 1;
			if (j == -1) j = polygons[arr[i][1]].length - 1;
			if (!VisibilityPolygon.equal(cur, polygons[arr[i][1]][k])) continue;
			var a1 = VisibilityPolygon.angle2(polygons[arr[i][1]][j], polygons[arr[i][1]][k], position);
			if (a1 <= 0 || a1 >= 180) continue;
			if (a2 <= 0 || a2 >= 180 || a2 < a1) {
				if (a1 > best_a) {
					best_a = a1;
					next[0] = arr[i][1];
					next[1] = j;
				}
			}
		}
	}
};

VisibilityPolygon.extend = function(cur, next, polygons, position) {
	var bestDis = -1;
	var best = [0, 0];
	var bestPoly = 0;
	var bestIndex = 0;
	var a2 = VisibilityPolygon.angle(cur, position);
	for (var j = 0; j < polygons.length; ++j) {
		var segments = VisibilityPolygon.getSegments(polygons[j], a2, position);
		for (var k = 0; k < segments.length; ++k) {
			if (VisibilityPolygon.equal(cur, polygons[j][segments[k][0]]) || VisibilityPolygon.equal(cur, polygons[j][segments[k][1]])) continue;
			var a3 = 0;
			if (j == 0) a3 = VisibilityPolygon.angle2(polygons[j][segments[k][1]], polygons[j][segments[k][0]], position);
			else a3 = VisibilityPolygon.angle2(polygons[j][segments[k][0]], polygons[j][segments[k][1]], position);
			if (a3 <= 0 || a3 >= 180) continue;
			var intersect = VisibilityPolygon.intersectLines(cur, position, polygons[j][segments[k][0]], polygons[j][segments[k][1]]);
			if (intersect.length > 0) {
				dis = VisibilityPolygon.distance(intersect, position);
				if (dis < VisibilityPolygon.distance(position, cur)) continue;
				if (dis < bestDis || bestDis == -1) {
					bestDis = dis;
					best = intersect;
					bestPoly = j;
					bestIndex = segments[k][0];
				}
			}
		}
	}
	next[0] = bestPoly;
	next[1] = bestIndex;
	if (next[0] == 0) {
		next[1] = next[1] + 1;
		if (next[1] == polygons[next[0]].length) next[1] = 0;
	}
	return best;
};

VisibilityPolygon.trace = function(cur, next, sorted, polygons, position, polygon) {
	var start = 0;
	var a = VisibilityPolygon.angle(cur, position);
	var bot = 0;
	var top = sorted.length-1;
	if (sorted[0][3] + VisibilityPolygon.epsilon() > a) {
		start = 0;
	} else if (sorted[top][3] < a + VisibilityPolygon.epsilon()) {
		start = top;
	} else {
		while (true) {
			var mid = bot + Math.floor((top-bot) / 2);
			if (bot == mid) {
				start = bot;
				break;
			}
			if (VisibilityPolygon.equal(sorted[mid][0], cur)) {
				start = mid;
				break;
			}
			if (sorted[mid][3] < a + VisibilityPolygon.epsilon()) {
				bot = mid;
				continue;
			}
			top = mid;
		}
	}
	for (var k = 0; k < sorted.length; ++k) {
		var j = (k + start) % sorted.length;
		if (VisibilityPolygon.equal(sorted[j][0], cur)) continue;
		if (VisibilityPolygon.equal(sorted[j][0], polygons[next[0]][next[1]])) break;
		var a1 = sorted[j][3];
		var a2 = VisibilityPolygon.angle(cur, position);
		var a3 = VisibilityPolygon.angle(polygons[next[0]][next[1]], position);
		if (a2 < -90 && a3 > 90) {
			a2 += 360;
			if (a1 < -90) a1 += 360;
		}
		if (a3 < -90 && a2 > 90) {
			a3 += 360;
			if (a1 < -90) a1 += 360;
		}
		if ((a1 > a2 + VisibilityPolygon.epsilon() && a1 <= a3 && a3 - a2 < 180) || (a1 >= a3 && a1 < a2 - VisibilityPolygon.epsilon() && a2 - a3 < 180)) {
			var intersect = VisibilityPolygon.intersectLines(sorted[j][0], position, cur, polygons[next[0]][next[1]]);
			if (intersect.length > 0) {
				var d1 = VisibilityPolygon.distance(position, intersect);
				var d2 = VisibilityPolygon.distance(position, sorted[j][0]);
				if (d2 < d1) {
					polygon.push(intersect);
					next[0] = sorted[j][1];
					next[1] = sorted[j][2];
					break;
				}
			}
		}
	}
	cur = polygons[next[0]][next[1]];
	VisibilityPolygon.increment(next, polygons[next[0]].length);
	return cur;
};

VisibilityPolygon.increment = function(next, len) {
	if (next[0] == 0) {
		next[1] = next[1] + 1;
		if (next[1] == len) next[1] = 0;
	} else {
		next[1] = next[1] - 1;
		if (next[1] == -1) next[1] = len - 1;
	}
};

VisibilityPolygon.equal = function(a, b) {
	if (Math.abs(a[0] - b[0]) < VisibilityPolygon.epsilon() && Math.abs(a[1] - b[1]) < VisibilityPolygon.epsilon()) return true;
	return false;
};

VisibilityPolygon.getSegments = function(polygon, a, position) {
	var segments = [];
	for (var i = 0; i < polygon.length; ++i) {
		var j = i+1;
		if (j == polygon.length) j = 0;
		var a1 = a;
		var a2 = VisibilityPolygon.angle(polygon[i], position);
		var a3 = VisibilityPolygon.angle(polygon[j], position);
		if (a2 < -90 && a3 > 90) {
			a2 += 360;
			if (a < -90) a1 += 360;
		}
		if (a3 < -90 && a2 > 90) {
			a3 += 360;
			if (a < -90) a1 += 360;
		}
		if ((a1 >= a2 - VisibilityPolygon.epsilon() && a1 <= a3 + VisibilityPolygon.epsilon() && a3 - a2 < 180) ||
				(a1 >= a3 - VisibilityPolygon.epsilon() && a1 <= a2 + VisibilityPolygon.epsilon() && a2 - a3 < 180)) {
			segments.push([i,j]);
		}
	}
	return segments;
};

VisibilityPolygon.sortPoints = function(position, polygons) {
	var points = [];
	for (var i = 0; i < polygons.length; ++i) {
		for (var j = 0; j < polygons[i].length; ++j) {
			points.push([polygons[i][j], i, j, VisibilityPolygon.angle(polygons[i][j], position)]);
		}
	}
	points.sort(
		function(a,b) {
			if (a[3] == b[3]) return VisibilityPolygon.distance(a[0], position) - VisibilityPolygon.distance(b[0], position);
			return a[3]-b[3];
		});
	return points;
};

VisibilityPolygon.makeMap = function(polygons) {
	var map = {};
	var points = [];
	for (var i = 0; i < polygons.length; ++i) {
		for (var j = 0; j < polygons[i].length; ++j) {
			var key = polygons[i][j][0] + "," + polygons[i][j][1];
			if (!(key in map)) {
				map[key] = [];
			}
			map[key].push([polygons[i][j], i, j]);
		}
	}
	return map;
};

VisibilityPolygon.angle = function(a, b) {
	return Math.atan2(b[1]-a[1], b[0]-a[0]) * 180 / Math.PI;
};

VisibilityPolygon.angle2 = function(a, b, c) {
	var a1 = VisibilityPolygon.angle(a,b);
	if (a1 < 0) a1 += 360;
	var a2 = VisibilityPolygon.angle(b,c);
	var a = a1 - a2;
	if (a > 360) a -= 360;
	return a;
};

VisibilityPolygon.firstVisibleVertex = function(position, polygons) {
	var closestDis = -1;
	var closest = [];
	var poly = 0;
	var index = 0;
	for (var i = 0; i < polygons.length; ++i) {
		for (var j = 0; j < polygons[i].length; ++j) {
			var index2 = 0;
			if (i == 0) {
				index2 = j-1;
				if (index2 == -1) index2 = polygons[i].length - 1;
			} else {
				index2 = j+1;
				if (index2 == polygons[i].length) index2 = 0;
			}
			var p = VisibilityPolygon.closestPoint(polygons[i][j], polygons[i][index2], position);
			var dis = VisibilityPolygon.distance(p, position);
			if (closestDis == -1 || dis < closestDis) {
				closestDis = dis;
				closest = p;
				poly = i;
				index = j;
			}
		}
	}
	return [closest, [poly, index]];
};

VisibilityPolygon.isOnSegment = function(xi, yi, xj, yj, xk, yk) {
  return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
         (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
};

VisibilityPolygon.computeDirection = function(xi, yi, xj, yj, xk, yk) {
  a = (xk - xi) * (yj - yi);
  b = (xj - xi) * (yk - yi);
  return a < b ? -1 : a > b ? 1 : 0;
};

VisibilityPolygon.doLineSegmentsIntersect = function(x1, y1, x2, y2, x3, y3, x4, y4) {
  d1 = VisibilityPolygon.computeDirection(x3, y3, x4, y4, x1, y1);
  d2 = VisibilityPolygon.computeDirection(x3, y3, x4, y4, x2, y2);
  d3 = VisibilityPolygon.computeDirection(x1, y1, x2, y2, x3, y3);
  d4 = VisibilityPolygon.computeDirection(x1, y1, x2, y2, x4, y4);
  return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
          ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
         (d1 == 0 && VisibilityPolygon.isOnSegment(x3, y3, x4, y4, x1, y1)) ||
         (d2 == 0 && VisibilityPolygon.isOnSegment(x3, y3, x4, y4, x2, y2)) ||
         (d3 == 0 && VisibilityPolygon.isOnSegment(x1, y1, x2, y2, x3, y3)) ||
         (d4 == 0 && VisibilityPolygon.isOnSegment(x1, y1, x2, y2, x4, y4));
};

VisibilityPolygon.intersectLines = function(a1, a2, b1, b2) {
	var ua_t = (b2[0] - b1[0]) * (a1[1] - b1[1]) - (b2[1] - b1[1]) * (a1[0] - b1[0]);
	var ub_t = (a2[0] - a1[0]) * (a1[1] - b1[1]) - (a2[1] - a1[1]) * (a1[0] - b1[0]);
	var u_b  = (b2[1] - b1[1]) * (a2[0] - a1[0]) - (b2[0] - b1[0]) * (a2[1] - a1[1]);
	if (u_b != 0) {
		var ua = ua_t / u_b;
		var ub = ub_t / u_b;
		return [a1[0] - ua * (a1[0] - a2[0]), a1[1] - ua * (a1[1] - a2[1])];
	}
	return [];
};

VisibilityPolygon.closestPoint = function(lineSegmentStart, lineSegmentEnd, point) {
	var t1 = [point[0] - lineSegmentStart[0], point[1] - lineSegmentStart[1]];
	var t2 = [lineSegmentEnd[0] - lineSegmentStart[0], lineSegmentEnd[1] - lineSegmentStart[1]];
	var t3 = (t1[0]*t2[0] + t1[1]*t2[1]) / (t2[0]*t2[0] + t2[1]*t2[1]);
	var p = [lineSegmentStart[0] + t2[0]*t3, lineSegmentStart[1] + t2[1]*t3];
	var d1 = VisibilityPolygon.distance(lineSegmentStart, p);
	var d2 = VisibilityPolygon.distance(lineSegmentEnd, p);
	var d3 = VisibilityPolygon.distance(lineSegmentStart, lineSegmentEnd);
	if (d1 + d2 <= d3) return p;
	var d4 = VisibilityPolygon.distance(lineSegmentStart, point);
	var d5 = VisibilityPolygon.distance(lineSegmentEnd, point);
	if (d4 < d5) return lineSegmentStart;
	return lineSegmentEnd;
};

VisibilityPolygon.distance = function(a, b) {
	return (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]);
};
