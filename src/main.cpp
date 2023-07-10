#include <map>
#include <set>
#include <vector>
#include <deque>
#include <iostream>
#include <algorithm>
#include "MazeBot.h"
#include <string>

int left_mot[2]={16,17},right_mot[2]={13,19},ultrasonic_echo[4]={27,18,14,33},ultrasonic_trig=5;
HCSR04 ultrasonic(ultrasonic_trig,ultrasonic_echo,4);
MPU6050 mpu;

MazeBot mazebot(left_mot,right_mot,ultrasonic_echo,ultrasonic_trig);


enum DIRECTIONS {
    EAST = 1, SOUTH = 2, WEST = 3, NORTH = 4
};

typedef struct Point {
    int x;
    int y;

    bool operator==(const Point &rhs) const {
        if(x==rhs.x && y==rhs.y){
        return true;
        }
        return false;
    }

    // bool operator<=>(const Point &rhs) const {
    //     if (x < rhs.x)
    //         return true;
    //     if (rhs.x < x)
    //         return false;
    //     return y < rhs.y;
    // }

    bool operator<(const Point &rhs) const {
        if (x < rhs.x)
            return true;
        if (rhs.x < x)
            return false;
        return y < rhs.y;
    }

} PointType;

class Mouse {
public:
    virtual ~Mouse() = default;

    virtual int maze_width() = 0;

    virtual int maze_height() = 0;

    virtual bool wall_front() = 0;

    virtual bool wall_right() = 0;

    virtual bool wall_left() = 0;

    virtual void move_forward() = 0;

    virtual void turn_right() = 0;

    virtual void turn_left() = 0;

    virtual void ack_reset() = 0;
};

class CustomMouse : public Mouse {
    int maze_width() override {
        return 16;
    }

    int maze_height() override {
        return 16;
    }

    bool wall_front() override {
        return mazebot.wall_front();
    }

    bool wall_right() override {
        return mazebot.wall_right();
    }

    bool wall_left() override {
        return mazebot.wall_left();
    }

    void move_forward() override {
        mazebot.forward();
    }

    void turn_right() override {
        mazebot.turn_right();
    }

    void turn_left() override {
        mazebot.turn_left();
    }

    void ack_reset() override {
        ;
    }
};

class [[maybe_unused]] FloodFill {
public:
    Mouse *mouse;
    int w;
    int h;
    std::map<int, std::vector<PointType>> paths{};
    std::map<DIRECTIONS, PointType> directions{
            {EAST,  {0,  1}},
            {SOUTH, {1,  0}},
            {WEST,  {0,  -1}},
            {NORTH, {-1, 0}}
    };
    DIRECTIONS orient{EAST};
    PointType current{0, 0};
    std::deque<PointType> queue{};
    std::map<PointType, std::vector<PointType>> walls;
    std::vector<std::vector<int>> cells{{0}};
    std::vector<std::vector<int>> flood{{0}};

    [[maybe_unused]] explicit FloodFill(Mouse *mouse) : mouse(mouse), w(mouse->maze_width()), h(mouse->maze_height()) {
        generateFloodMatrix();
    }

    [[nodiscard]] std::vector<PointType> neighbours(PointType point) const {
        std::vector<PointType> neighbours{};
        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                PointType neighbour{point.x + i, point.y + j};
                if (neighbour.x >= 0 && neighbour.x < w && neighbour.y >= 0 && neighbour.y < h && !(i && j) &&
                    (i || j)) {
                    neighbours.push_back(neighbour);
                }
            }
        }
        return neighbours;
    }

    void generateFloodMatrix() {
        // double midpoint = (w - 1) / 2.0;
        // std::vector<std::vector<int>> matrix;
        // for (int y = 0; y < h; ++y) {
        //     std::vector<int> row;
        //     for (int x = 0; x < w; ++x) {
        //         auto val = static_cast<int>(midpoint - std::abs(x - midpoint) + std::max(0, static_cast<int>(y - midpoint)));
        //         row.push_back(val);
        //     }
        //     auto midRow = std::vector<int>(row.begin(), row.begin() + static_cast<int>(midpoint) + 1);
        //     std::ranges::reverse(midRow);
        //     midRow.insert(midRow.end(), row.begin(), row.begin() + static_cast<int>(midpoint) + 1);
        //     matrix.push_back(midRow);
        // }
        // std::ranges::reverse(matrix);
        // auto midMatrix = std::vector<std::vector<int>>(matrix.begin(), matrix.begin() + static_cast<int>(midpoint) + 1);
        // auto reversedMidMatrix = std::vector<std::vector<int>>(matrix.begin(), matrix.begin() + static_cast<int>(midpoint) + 1);
        // midMatrix.insert(midMatrix.end(), reversedMidMatrix.rbegin(), reversedMidMatrix.rend());
        flood = {
{14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14},
{13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
{12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
{11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
{10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
{9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
{8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
{7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
{7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
{8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
{9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
{10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
{11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
{12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
{13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
{14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14}
};
    }

    void updateWalls() {
        log("wall list size:"+std::to_string(walls.size()));
        bool left_wall = mouse->wall_left();
        bool right_wall = mouse->wall_right();
        bool front_wall = mouse->wall_front();
        std::set<DIRECTIONS> wallSet{};
        if (left_wall) {
            wallSet.insert(DIRECTIONS((orient > 1) ? orient - 1 : 4));
        }
        if (front_wall) {
            wallSet.insert(orient);
        }
        if (right_wall) {
            wallSet.insert(DIRECTIONS((orient < 4) ? orient + 1 : 1));
        }
        if (!walls.contains(current)) {
            walls[current] = {};
        }
        std::vector<PointType> wallPoints{walls[current]}; //{walls[current]} --removed
        for (auto &wall: wallSet) {
            int dx{directions[wall].x};
            int dy{directions[wall].y};
            PointType wallPoint{current.x + dx, current.y + dy};
            if (std::ranges::find(wallPoints, wallPoint) == wallPoints.end() && wallPoint.x >= 0 &&
                wallPoint.x < w && wallPoint.y >= 0 && wallPoint.y < h) {
                wallPoints.push_back(wallPoint);
            }
        }
        for(const auto &wall: wallPoints){
            if (!walls.contains(wall)) {
                walls[wall] = {current};
        }
        }
        walls[current]=wallPoints; //added
        // int i;
        // log("walls added("+std::to_string(current.x)+","+std::to_string(current.y)+"):",0);
        // for(i=0; i<wallPoints.size(); i++) {
        // log("("+std::to_string(walls[current][i].x)+","+std::to_string(walls[current][i].y)+")"+"  ",0);
        // }
        // log(" ");
        // log("updatewalls ended");
        // for(const auto& [key, value] : walls){
        // log("("+std::to_string(key.x)+","+std::to_string(key.y)+"):",0);
        // log("size:"+std::to_string(value.size()));
        // for(int i=0; i<value.size();i++){
        //     log("("+std::to_string(value[i].x)+","+std::to_string(value[i].y)+") ",0);
        // }
        // log(" ");

        // }
    }

    void floodfillAll(PointType cell) {
        queue.push_back(cell);
        log("called floodfill");
        for(const auto &[key, value] : walls){
        log("("+std::to_string(key.x)+","+std::to_string(key.y)+"):",0);
        for(int i=0; i<value.size();i++){
            log("("+std::to_string(value[i].x)+","+std::to_string(value[i].y)+") ",0);
        }
        log(" ");

        }
        while (!queue.empty()) {
            log("queue:",0);
            int i;
            for(i=0 ;i<queue.size(); i++) {
                log("("+std::to_string(queue[i].x)+","+std::to_string(queue[i].y)+"),",0);
            }
            log(" ");
            PointType popped_cell{queue.back()};
            queue.pop_back();
            if (!walls.contains(popped_cell)) {
                walls[popped_cell] = {};
            }
            std::vector<PointType> wallPoints{walls[popped_cell]}; 
            // for(i=0 ;i<wallPoints.size();i++) {
            //     log("floodfill:wallpoints:"+std::to_string(popped_cell.x)+","+std::to_string(popped_cell.y)+" pts:"+std::to_string(wallPoints[i].x)+","+std::to_string(wallPoints[i].y));
                

            // }
            std::vector<PointType> neighbourPoints{neighbours(popped_cell)};
            for (auto const &wall: wallPoints) {
                // log("floodfill:popped_cell:"+std::to_string(popped_cell.x)+","+std::to_string(popped_cell.y)+" erasing:"+std::to_string(wall.x)+","+std::to_string(wall.y));
                std::erase(neighbourPoints, wall);
            }
            std::vector<int> nValues{};
            nValues.reserve(neighbourPoints.size());
            for (auto const &neighbour: neighbourPoints) {
                nValues.push_back(flood[neighbour.x][neighbour.y]);
            }
            log("-------");
            for(int i = 0; i < neighbourPoints.size(); i++) {
                log("("+std::to_string(neighbourPoints[i].x)+","+std::to_string(neighbourPoints[i].y)+") ",0);
            }
            log(" ");
            for(i=0;i<nValues.size();i++) {
                log("  "+std::to_string(nValues[i])+"  ",0);
            }
            log("");
            log("------");
            int min{*std::ranges::min_element(nValues)};
            if (flood[popped_cell.x][popped_cell.y] <= min) {
                flood[popped_cell.x][popped_cell.y] = min + 1;
                for (auto const &neighbour: neighbourPoints) {
                    queue.push_back(neighbour);
                }
            }
        }
    }

    PointType findCell(PointType cell, std::vector<PointType> ignore={}) {
        if (!walls.contains(cell)) {
            walls[cell] = {};
        }
        int i,j;
        std::vector<PointType> wallPoints{walls[cell]};
        std::vector<PointType> neighbourPoints{neighbours(cell)};
        
        for(i=0;i<ignore.size();i++){
            for(j=0;j<neighbourPoints.size();j++){
                // log("("+std::to_string(neighbourPoints[j].x)+","+std::to_string(neighbourPoints[j].y)+")"+"  ",0);
                if((ignore[i].x==neighbourPoints[j].x) && (ignore[i].y==neighbourPoints[j].y)){
                    // log("ignore if called---------------------------------");
                    neighbourPoints.erase(neighbourPoints.begin()+j);
                }
            }

        }
        // log("neighbour points:",0);
        // for(i=0; i<neighbourPoints.size(); i++) {
        // log("("+std::to_string(neighbourPoints[i].x)+","+std::to_string(neighbourPoints[i].y)+")"+"  ",0);
        // };
        // log(" ");
        // for (auto const &wall: walls[cell]) {     //removed
        //     std::erase(neighbourPoints, wall);
        // }
        
        // log("acessible neighbour points:",0);
        // for(i=0; i<neighbourPoints.size(); i++) {
        // log("("+std::to_string(neighbourPoints[i].x)+","+std::to_string(neighbourPoints[i].y)+")"+"  ",0);
        // };
        // log(" ");
        std::vector<int> nValues{};
        nValues.reserve(neighbourPoints.size());
        for (auto const &neighbour: neighbourPoints) {
            nValues.push_back(flood[neighbour.x][neighbour.y]);
        }
        int min{*std::ranges::min_element(nValues)};
        std::vector<int> indices{};
        for (int i = 0; i < nValues.size(); i++) {
            if (nValues[i] == min) {
                indices.push_back(i);
            }
        }
        PointType dir{directions[orient]};
        std::vector<int> check{};
        for (auto const &index: indices) {
            PointType neighbour{neighbourPoints[index]};
            PointType diff{neighbour.x - cell.x, neighbour.y - cell.y};
            if (diff.x == dir.x && diff.y == dir.y) {
                check.push_back(index);
            }
        }
        PointType nextCell{(check.empty()) ? neighbourPoints[indices[0]] : neighbourPoints[check[0]]};
        // log("findcell:Next cell:"+std::to_string(nextCell.x)+","+std::to_string(nextCell.y));
        if (std::ranges::find(wallPoints, nextCell) == wallPoints.end()) {
            // log("Not a wall");
            if (flood[cell.x][cell.y] <= flood[nextCell.x][nextCell.y]) { 
                // log("Floodfilling");
                floodfillAll(cell);
            }
            return nextCell;
        }
        // log("Wall");
        floodfillAll(cell);
        ignore.push_back(nextCell);
        // log("ignore size:"+std::to_string(ignore.size()));
        // for(i=0;i<ignore.size();i++){
        //     log("Ignoring:"+std::to_string(ignore[i].x)+","+std::to_string(ignore[i].y)+" ",0);
        // }
        return findCell(cell, ignore);
    }


    void move(PointType cell) {
        PointType diff{cell.x - current.x, cell.y - current.y};   //EAST = 1, SOUTH = 2, WEST = 3, NORTH = 4
        // log("diff:"+std::to_string(diff.x)+","+std::to_string(diff.y));
        /*{EAST,  {0,  1}},
          {SOUTH, {1,  0}},
          {WEST,  {0,  -1}},
          {NORTH, {-1, 0}}*/
        DIRECTIONS nextOrient{};
        for (auto const &[key, value]: directions) {
            if (value.x == diff.x && value.y == diff.y) {
                nextOrient = key;
                // log("next orient:"+std::to_string(value.x)+","+std::to_string(value.y));
                // log("orient:"+std::to_string(orient));
            }
        }
        int orientDiff=(nextOrient - orient);
        if(orientDiff<0){
            orientDiff+=4;
        }
        // log("orientDiff:"+std::to_string(orientDiff));
        DIRECTIONS check{};
        for (int i = 1; i < 5; i++) {
            if (i == orientDiff) {
                check = DIRECTIONS(i);
                // log("finding check:"+std::to_string(i));
            }
        }
        if(check==0){
            check = NORTH;
        }
        // log("check:");
        // log(std::to_string(check));

        switch (check) {
            case NORTH:
                mouse->move_forward();
                break;
            case EAST:
                mouse->turn_right();
                mouse->move_forward();
                break;
            case SOUTH:
                mouse->turn_right();
                mouse->turn_right();
                mouse->move_forward();
                break;
            case WEST:
                mouse->turn_left();
                mouse->move_forward();
                break;
        }
        current = cell;
        orient = DIRECTIONS((orient + orientDiff)>4?(orient+orientDiff-4):(orient+orientDiff));
    }

    static std::vector<PointType> cutRedundantSteps(std::vector<PointType> path) {
        std::vector<PointType> newPath{};
        while (!path.empty()) {
            PointType current{path.back()};
            path.pop_back();
            if (std::ranges::find(path, current) != path.end()) {
                std::vector<PointType> tempPath{path};
                std::ranges::reverse(tempPath);
                int index{static_cast<int>(std::ranges::find(tempPath, current) - tempPath.begin())};
                path = std::vector<PointType>(path.begin() + static_cast<int>(path.size()) - index, path.end());
            }
            newPath.push_back(current);
        }
        return newPath;
    }

    void run() {
        std::vector<PointType> steps{};
        log("running");
        int i,j;
        while (flood[current.x][current.y]) {
            updateWalls();
            PointType nextCell{findCell(current, {})};
            std::string s;
            // s+=std::to_string(nextCell.x);;s+=",";s+=std::to_string(nextCell.y);
            // log("nextCell:"+s);           
            for(i=0;i<flood.size();i++){
                for(j=0;j<flood[i].size();j++){
                    log(std::to_string(flood[i][j])+",",0);
                }
                log(" ");
            }
            move(nextCell);
            steps.push_back(current);
        }
        steps = cutRedundantSteps(steps);
        paths[static_cast<int>(steps.size())] = steps;
    }

    [[maybe_unused]] void reset(bool manual) {
        if (manual) {
            mouse->ack_reset();
            current = {0, 0};
            orient = EAST;
            return;
        }
        if (!paths.empty()) {
            auto min = std::ranges::min_element(paths);
            std::vector<PointType> steps{min->second};
            std::ranges::reverse(steps);
            for (auto const &step: std::vector<PointType>(steps.begin(), steps.end() - 1)) {
                move(step);
            }
            move({0, 0});
        }
    }

    void log(const std::string& text, int i=1) {
        if(i == 0) {
        std::cerr << text;
    }
    else{
        std::cerr << text << std::endl;
    }
    }
};

void setup() {
    analogWrite(0,0);
    pinMode(27,OUTPUT);
    pinMode(26,OUTPUT);
    pinMode(32,OUTPUT);
  Serial.begin(115200);
  Serial.println("setup_arnav");
  mazebot.begin();
  Serial.println("setup");
  mazebot.proportionality_const=10;
  pinMode(39,INPUT);
  pinMode(36,INPUT);
  digitalWrite(27,1);
  digitalWrite(26,1);
  delay(1000);

}

void loop() {
    CustomMouse cantbemouse{};
    FloodFill floodfill{&cantbemouse};
    floodfill.run();
    floodfill.reset(false);
    delay(10000);
}