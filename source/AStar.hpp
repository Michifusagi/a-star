/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>

namespace AStar
{
    struct Vec2i
    {
        int x, y;

        // Vec2i向けに==を再定義
        bool operator == (const Vec2i& coordinates_);

        // Vec2i向けに+を再定義
        // x, yはpublicなのでfriendで宣言する必要はない
        //　メンバ関数として定義してしてしまうと左側に型の制限がかかる
        friend Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x + right_.x, left_.y + right_.y };
        }
    };

    // usingでエイリアスを定義（別名）
    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        //　メンバ変数
        // スタートから現在までの実コストG
        // 現在からゴールまでの推定コストH
        // 現在の座標coordinates
        // 親ノードのポインタparent
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        // コンストラクタ宣言
        // デフォルトでは親ノードのポインタはnullptr
        Node(Vec2i coord_, Node *parent_ = nullptr);
        // スコアを取得するメンバ関数（cppファイルで定義）
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        // プライベートメンバ関数（内部だけで呼ばれる補助関数）

        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    // パブリックメンバ関数（外部インターフェース）
    public:
        // コンストラクタの宣言（変数の初期化）
        // publicで宣言しないと外部からインスタンス化できない
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();

    // プライベートメンバ（クラスの状態）
    private:
        // HeuristicFunction = std::function<uint(Vec2i, Vec2i)>
        // 評価関数
        HeuristicFunction heuristic;
        // CoordinateList = std::vector<Vec2i>
        // direction: 方向ベクトルの配列
        // walls: 障害物座標の配列
        CoordinateList direction, walls;
        // グリッドサイズ
        Vec2i worldSize;
        // 使用する方向数
        uint directions;
    };

    // 静的ユーティリティクラス（staticメンバ関数のみ）
    // インスタンス化せずに使用
    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
