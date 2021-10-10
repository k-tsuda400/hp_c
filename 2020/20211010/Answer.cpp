// HPC2020 公式サイト：
// https://www.hallab.co.jp/progcon/2020/

#include "Answer.hpp"
#include <vector>

//------------------------------------------------------------------------------
namespace hpc {

//------------------------------------------------------------------------------
/// コンストラクタ
/// @detail 最初のステージ開始前に実行したい処理があればここに書きます
Answer::Answer()
{
}

//------------------------------------------------------------------------------
/// デストラクタ
/// @detail 最後のステージ終了後に実行したい処理があればここに書きます
Answer::~Answer()
{
}

//------------------------------------------------------------------------------
/// 各ステージ開始時に呼び出される処理
/// @detail 各ステージに対する初期化処理が必要ならここに書きます
/// @param aStage 現在のステージ
void Answer::initialize(const Stage& aStage)
{
}

//------------------------------------------------------------------------------
/// 毎フレーム呼び出される処理
/// @detail 移動先を決定して返します
/// @param aStage 現在のステージ
/// @return 移動の目標座標
Vector2 Answer::getTargetPos(const Stage& aStage)
{
    auto rabPos = aStage.rabbit().pos();

	float nearestDistanceToScr = 99999999.0;
	int nearestDistanceToScrIdx = 0;

	int scrIdx = 0;
	for (auto scroll : aStage.scrolls()) {
		// 最も近くかつ未取得の巻物を探す
		float distance = (scroll.pos().x - rabPos.x) * (scroll.pos().x - rabPos.x) +
						 (scroll.pos().y - rabPos.y) * (scroll.pos().y - rabPos.y);
		if (nearestDistanceToScr > distance && !scroll.isGotten()) {
			nearestDistanceToScrIdx = scrIdx;
			nearestDistanceToScr = distance;
		}
		++scrIdx;
	}

	scrIdx = 0;
    for(auto scroll : aStage.scrolls()) {

        // 最も近くかつ未取得の巻物に向かって飛ぶ
		if (scrIdx == nearestDistanceToScrIdx && !scroll.isGotten()){
            return scroll.pos();
        }

		scrIdx = scrIdx + 1;
    }

    return rabPos;
}

//------------------------------------------------------------------------------
/// 各ステージ終了時に呼び出される処理
/// @detail 各ステージに対する終了処理が必要ならここに書きます
/// @param aStage 現在のステージ
void Answer::finalize(const Stage& aStage)
{
}

} // namespace
// EOF
