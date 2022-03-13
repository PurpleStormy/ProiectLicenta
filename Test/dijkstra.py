from collections import deque

class Solution:
    def shortestPath(self, grid, start_i, start_j, end_i, end_j):
        M, N = end_i, end_j
        visited = set()
        q = deque()
        dirs = [(0, 1), (0, -1), (1, 0), (-1, 0), (-1, -1), (1, 1), (-1, 1), (1, -1)]
        if grid[start_i][start_j] == 1:
            q.append((1, (start_i, start_j)))
            visited.add((start_i, start_j))
        while q:
            steps, tmp = q.popleft()
            r, c = tmp[0], tmp[1]
            if (r, c) == (M-1, N-1):
                print("visited: " + visited)
                print("queue: " + q)
                return steps
            for i, j in dirs:
                new_r, new_c = r+i, c+j
                if 0<=new_r<M and 0<=new_c<N and grid[new_r][new_c]==0 and (new_r, new_c) not in visited:
                    q.append((steps + 1, (new_r, new_c)))
                    visited.add((new_r, new_c))
        print("Visited:")
        print(visited)
        print("Queue:")
        print(q)
        return -1