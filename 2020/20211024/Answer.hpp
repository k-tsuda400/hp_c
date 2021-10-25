//--------------

#pragma once

//------------------------------------------------------------------------------
#include "Array.hpp"
#include "ArrayNum.hpp"
#include "Assert.hpp"
#include "Math.hpp"
#include "Print.hpp"
#include "Stage.hpp"
#include <vector>

//------------------------------------------------------------------------------
namespace hpc {

/// ゲームへの解答
/// @detail 参加者は Answer.cpp にこのクラスのメンバ関数を実装することで、
///         プログラムを作成してください
class Answer
{
public:
    /// @name コンストラクタ/デストラクタ
    //@{
    Answer();
    ~Answer();
    //@}

    /// @name 解答
    //@{
    /// 各ステージ開始時に呼び出される処理
    void initialize(const Stage& aStage);
	
	/// 各ターンの行動を決定する
    Vector2 getTargetPos(const Stage& aStage);
    /// 各ステージ終了時に呼び出される処理
    void finalize(const Stage& aStage);
    //@}
private:
	/// 歩数シミュレート用
	int calcSimuWalkNum(const Vector2 srcVec, const Vector2 tgtVec, const Vector2 dirVec, const int getScrNum, const Stage& aStage);
	/// 与えられたposに対する地形データを調べ、そのジャンプ力の比率を返す
	float convertTerrainToFloat(const Terrain terrain);
	/// ベクトル内に指定した整数が存在するか否かを返す
	bool vector_finder(std::vector<int> vec, int number);
};

} // namespace
// EOF
