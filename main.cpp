#include <Novice.h>
#include <Novice.h>
#include <imgui.h>
#include "MakeMatrix.h"
#include "MatrixMath.h"
#include "Vector3Math.h"
#include "Draw.h"
#include "algorithm"

const char kWindowTitle[] = "LE2B_17_ナガイ_コハク_MT3_2_9 OBBと線の当たり判定";

bool IsCollision(OBB obb, Segment segment);

bool IsCollision(AABB aabb, Segment segment);

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	//ウィンドウサイズ
	float kWindowWidth = 1280.0f;
	float kWindowHeight = 720.0f;

	//カメラ:平行移動
	Vector3 cameraTranslate{ 0.0f,1.9f,-6.49f };

	//カメラ:回転
	Vector3 cameraRotate{ 0.26f,0.0f,0.0f };

	Vector3 rotate{ 0.0f,0.0f,0.0f };

	OBB obb{
		.center{-1.0f,0.0f,0.0f},
		.orientations =
		{
			{1.0f,0.0f,0.0f},
			{0.0f,1.0f,0.0f},
			{0.0f,0.0f,1.0f}
		},
		.size{0.5f,0.5f,0.5f}
	};

	Segment segment{
		.origin{-0.8f,-0.3f,0.0f},
		.diff{0.5f,0.5f,0.5f}
	};

	Draw draw;

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		if (ImGui::TreeNode("Camera")) {
			ImGui::DragFloat3("Translate", &cameraTranslate.x, 0.01f);
			ImGui::DragFloat3("Rotate", &cameraRotate.x, 0.01f);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("OBB")) {
			ImGui::DragFloat3("orientations[0]", &obb.orientations[0].x, 0.01f);
			ImGui::DragFloat3("orientations[1]", &obb.orientations[1].x, 0.01f);
			ImGui::DragFloat3("orientations[2]", &obb.orientations[2].x, 0.01f);
			ImGui::DragFloat3("center", &obb.center.x, 0.01f);
			ImGui::DragFloat3("size", &obb.size.x, 0.01f);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Segment")) {
			ImGui::DragFloat3("origin", &segment.origin.x, 0.01f);
			ImGui::DragFloat3("diff", &segment.diff.x, 0.01f);
			ImGui::TreePop();
		}

		ImGui::DragFloat3("rotate", &rotate.x, 0.01f);

		Matrix4x4 rotateMatrix = Multiply(MakeRotateXMatrix(rotate.x), Multiply(MakeRotateYMatrix(rotate.y), MakeRotateZMatrix(rotate.z)));

		obb.orientations[0].x = rotateMatrix.m[0][0];
		obb.orientations[0].y = rotateMatrix.m[0][1];
		obb.orientations[0].z = rotateMatrix.m[0][2];

		obb.orientations[1].x = rotateMatrix.m[1][0];
		obb.orientations[1].y = rotateMatrix.m[1][1];
		obb.orientations[1].z = rotateMatrix.m[1][2];

		obb.orientations[2].x = rotateMatrix.m[2][0];
		obb.orientations[2].y = rotateMatrix.m[2][1];
		obb.orientations[2].z = rotateMatrix.m[2][2];

		draw.Pipeline(cameraTranslate, cameraRotate, kWindowWidth, kWindowHeight);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		draw.DrawGrid();

		if (IsCollision(obb, segment)) {
			draw.DrawOBB(obb, RED);
		} else {
			draw.DrawOBB(obb, WHITE);
		}

		draw.DrawLine(segment.origin, segment.diff, WHITE);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}

bool IsCollision(OBB obb, Segment segment) {

	Matrix4x4 scaleMatrix = MakeScaleMatrix(Vector3{ 1.0f,1.0f,1.0f });

	Matrix4x4 rotationMatrix;
	rotationMatrix.m[0][0] = obb.orientations[0].x;
	rotationMatrix.m[0][1] = obb.orientations[0].y;
	rotationMatrix.m[0][2] = obb.orientations[0].z;
	rotationMatrix.m[0][3] = 0.0f;
	rotationMatrix.m[1][0] = obb.orientations[1].x;
	rotationMatrix.m[1][1] = obb.orientations[1].y;
	rotationMatrix.m[1][2] = obb.orientations[1].z;
	rotationMatrix.m[1][3] = 0.0f;
	rotationMatrix.m[2][0] = obb.orientations[2].x;
	rotationMatrix.m[2][1] = obb.orientations[2].y;
	rotationMatrix.m[2][2] = obb.orientations[2].z;
	rotationMatrix.m[2][3] = 0.0f;
	rotationMatrix.m[3][0] = 0.0f;
	rotationMatrix.m[3][1] = 0.0f;
	rotationMatrix.m[3][2] = 0.0f;
	rotationMatrix.m[3][3] = 1.0f;

	Matrix4x4 transformMatrix = MakeTranslateMatrix(obb.center);

	Matrix4x4 worldMatrix = Multiply(scaleMatrix, Multiply(rotationMatrix, transformMatrix));

	Matrix4x4 obbWorldMatrixInverse = Inverse(worldMatrix);
	
	Vector3 localOrigin = Transform(segment.origin, obbWorldMatrixInverse);

	Vector3 localDiff = Transform(
		Vector3
		{
			segment.origin.x + segment.diff.x,
			segment.origin.y + segment.diff.y,
			segment.origin.z + segment.diff.z
		},
		obbWorldMatrixInverse);

	AABB aabbOBBLocal{
		.min{-obb.size.x,-obb.size.y,-obb.size.z},
		.max{+obb.size.x,+obb.size.y,+obb.size.z}
	};

	Segment localSegment{
		.origin{localOrigin.x,localOrigin.y,localOrigin.z},
		.diff{localDiff.x - localOrigin.x,localDiff.y - localOrigin.y,localDiff.z - localOrigin.z}
	};

	return IsCollision(aabbOBBLocal, localSegment);
}

bool IsCollision(AABB aabb, Segment segment) {

	float tXMin = (aabb.min.x - segment.origin.x) / segment.diff.x;
	float tXMax = (aabb.max.x - segment.origin.x) / segment.diff.x;

	float tYMin = (aabb.min.y - segment.origin.y) / segment.diff.y;
	float tYMax = (aabb.max.y - segment.origin.y) / segment.diff.y;

	float tZMin = (aabb.min.z - segment.origin.z) / segment.diff.z;
	float tZMax = (aabb.max.z - segment.origin.z) / segment.diff.z;

	float tNearX = fminf(tXMin, tXMax);
	float tNearY = fminf(tYMin, tYMax);
	float tNearZ = fminf(tZMin, tZMax);

	float tFarX = fmaxf(tXMin, tXMax);
	float tFarY = fmaxf(tYMin, tYMax);
	float tFarZ = fmaxf(tZMin, tZMax);

	float tMin = fmaxf(fmaxf(tNearX, tNearY), tNearZ);
	float tMax = fminf(fminf(tFarX, tFarY), tFarZ);

	if (tMin <= tMax) {
		return true;
	}

	return false;
}