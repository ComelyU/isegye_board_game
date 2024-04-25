package com.example.isegyeboard.firebase

import android.content.Context
import android.util.Log
import com.google.firebase.messaging.FirebaseMessaging
import com.google.firebase.messaging.FirebaseMessagingService
import com.google.firebase.messaging.RemoteMessage

class PushMessage : FirebaseMessagingService() {

    companion object {
        private const val TAG = "FirebaseMessagingService"
    }

    // 이 디바이스에 새 토큰을 생성할 때 호출
    // 토큰은 앱이 재설치 되거나 데이터가 삭제되면 다시 생성.
    override fun onNewToken(token: String) {
        super.onNewToken(token)
        Log.e(TAG, "NewToken: $token")

        // 토큰을 SharedPreferences 에 저장
        val pref = this.getSharedPreferences("token", Context.MODE_PRIVATE)
        val editor = pref.edit()
        editor.putString("token", token).apply()
        editor.commit()
        Log.i(TAG, "토큰 저장됨")
    }

    // 메세지를 받을때 호출
    // Notification 을 구현해준다.
    override fun onMessageReceived(remoteMessage: RemoteMessage) {
        super.onMessageReceived(remoteMessage)

        Log.d(TAG, "Message: ${remoteMessage.data}")

        if (remoteMessage.data.isEmpty()) {
            Log.e(TAG, "empty message")
        } else {
            showNotification(applicationContext, remoteMessage.data["body"] ?: "game")
        }
    }

    private fun showNotification(context: Context, message: String) {
        val channelId = "deliboard"
        val channelName = "딜리보드"
        var korean = "주문하신 게임이 도착했어요!"

        if (message == "2") {
            korean = "주문하신 음료가 도착했어요!"
        } else if (message == "1") {
            korean = "반납하실 게임패키지를 딜리부기에 올려놔 주세요."
        }
    }

    // 기기의 토큰을 가져오는 함수
    fun getFirebaseToken() {
        FirebaseMessaging.getInstance().token.addOnCompleteListener { task ->
            if (!task.isSuccessful) {
                Log.w(TAG, "Fetching FCM registration token failed", task.exception)
                return@addOnCompleteListener
            }

            // Get new FCM registration token
            val token = task.result

            // Log and handle the token as needed
            Log.d(TAG, "Token : $token")
        }
    }
}