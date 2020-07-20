var Util       = require('../core/Util');
var Heuristic  = require('../core/Heuristic');
var Node       = require('../core/Node');
var DiagonalMovement = require('../core/DiagonalMovement');

/**
 * Iterative Deeping A Star (IDA*) path-finder.
 *
 * Recursion based on:
 *   http://www.apl.jhu.edu/~hall/AI-Programming/IDA-Star.html
 *
 * Path retracing based on:
 *  V. Nageshwara Rao, Vipin Kumar and K. Ramesh
 *  "A Parallel Implementation of Iterative-Deeping-A*", January 1987.
 *  ftp://ftp.cs.utexas.edu/.snapshot/hourly.1/pub/AI-Lab/tech-reports/UT-AI-TR-87-46.pdf
 *
 * @author Gerard Meier (www.gerardmeier.com)
 *
 * @constructor
 * @param {Object} opt
 * @param {boolean} opt.allowDiagonal Whether diagonal movement is allowed.
 *     Deprecated, use diagonalMovement instead.
 * @param {boolean} opt.dontCrossCorners Disallow diagonal movement touching
 *     block corners. Deprecated, use diagonalMovement instead.
 * @param {DiagonalMovement} opt.diagonalMovement Allowed diagonal movement.
 * @param {function} opt.heuristic Heuristic function to estimate the distance
 *     (defaults to manhattan).
 * @param {number} opt.weight Weight to apply to the heuristic to allow for
 *     suboptimal paths, in order to speed up the search.
 * @param {boolean} opt.trackRecursion Whether to track recursion for
 *     statistical purposes.
 * @param {number} opt.timeLimit Maximum execution time. Use <= 0 for infinite.
 */
function IDAStarFinder(opt) {
    opt = opt || {};
    this.allowDiagonal = opt.allowDiagonal;
    this.dontCrossCorners = opt.dontCrossCorners;
    this.diagonalMovement = opt.diagonalMovement;
    this.heuristic = opt.heuristic || Heuristic.manhattan;
    this.weight = opt.weight || 1;
    this.trackRecursion = opt.trackRecursion || false;
    this.timeLimit = opt.timeLimit || Infinity; // Default: no time limit.

    if (!this.diagonalMovement) {
        if (!this.allowDiagonal) {
            this.diagonalMovement = DiagonalMovement.Never;
        } else {
            if (this.dontCrossCorners) {
                this.diagonalMovement = DiagonalMovement.OnlyWhenNoObstacles;
            } else {
                this.diagonalMovement = DiagonalMovement.IfAtMostOneObstacle;
            }
        }
    }

    if (this.diagonalMovement === DiagonalMovement.Never) {
        this.heuristic = opt.heuristic || Heuristic.manhattan;
    } else {
        this.heuristic = opt.heuristic || Heuristic.octile;
    }

}

/**
 * Find and return the the path. When an empty array is returned, either
 * no path is possible, or the maximum execution time is reached.
 *
 * @return {Array<Array<number>>} The path, including both start and
 *     end positions.
*/

IDAStarFinder.prototype.findPath = function(startX, startY, endX, endY, grid){

    var visitedNode = 0;

    var startTime = new Date().getTime();

    var h = function(a,b){
        return this.heuristic(Math.abs(b.x-a.x), Math.abs(b.y-a.y));
    }.bind(this);

    var cost = function(a,b){
        return (a.x===b.x || a.y===b.y)? 1 : Math.SQRT2 ;
    };

    /**
     * IDA* search implementation.
     *
     * @param {Node} The node currently expanding from.
     * @param {number} Cost to reach the given node.
     * @param {number} Maximum search depth (cut-off value).
     * @param {Array<Array<number>>} The found route.
     * @param {number} Recursion depth.
     *
     * @return {Object} either a number with the new optimal cut-off depth,
     * or a valid node instance, in which case a path was found.
     */

    var search = function(node, g, cutoff, route, depth){
         visitedNode++;

         if(this.timeLimit > 0 &&
            new Date().getTime() - startTime > this.timeLimit*1000) return Infinity;

         var f = g + h(node, end)*this.weight;

         if(f> cutoff) return f;

         if(node == end) {
             route[depth] = [node.x, node.y];
             return node;
         }

         var neighbor, neighbors, i, min, t;

         neighbors = grid.getNeighbors(node, this.diagonalMovement);

//       for(i=0;i<neighbors.length;++i)
         for(i=0, min=Infinity; neighbor = neighbors[i];
            ++i){
              
              if(this.trackRecursion){

                  neighbor.retainCount = neighbor.retainCount +1 || 1;

                  if(neighbor.tested !== true){ neighbor.tested = true;}
              }

              t = search(neighbor, g+cost(node,neighbor), cutoff, route, depth+1);

              if(t instanceof Node){
                  route[depth] = [node.x, node.y];
                  return t;
              }

              if(this.trackRecursion && (--neighbor.retainCount) === 0)  { neighbor.tested = false;}

              if(t< min) { min = t;}

         }

         return min;
    }.bind(this);

    var start = grid.getNodeAt(startX, startY);
    var end = grid.getNodeAt(endX, endY);

    var route, cutoff = h(start, end), j, t;

    for(j=0; true; j++){
       route = [];

       t = search(start, 0, cutoff, route, 0);

       if(t === Infinity) return [];

       if(t instanceof Node) return route;

       cutoff = t;
    }

    return [];
 };


module.exports = IDAStarFinder;
