package com.example.isegyeboard.room_login

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.util.Log
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.constraintlayout.widget.ConstraintLayout
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.ShowDialog
import com.example.isegyeboard.firebase.PushMessage
import com.example.isegyeboard.main_page.MainActivity
import com.google.android.material.textfield.TextInputEditText
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class InitialActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_initial)

        if (isLogined()) {
            val intent = Intent(this, MainActivity::class.java)
            startActivity(intent)
        }
//        else {
//            setContentView(R.layout.activity_initial)
////            val intent = Intent(this, MainActivity::class.java)
////            startActivity(intent)
//        }

        PushMessage().getFirebaseToken()

        val loginButton = findViewById<ConstraintLayout>(R.id.loginButton)
        val storeNumInput = findViewById<TextInputEditText>(R.id.storeNumField)
        val roomNumInput = findViewById<TextInputEditText>(R.id.roomNumField)

        loginButton.setOnClickListener {
            val storeId = storeNumInput.text.toString()
            val roomNum = roomNumInput.text.toString()

            storeNumCheck(storeId, roomNum)
        }
    }

    private fun isLogined() : Boolean {
        val sharedPreferences = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        return sharedPreferences.contains("roomId")
    }

    private fun storeNumCheck(storeId: String, roomNum: String) {
        val client = BaseApi.getInstance().create(LoginApi::class.java)

//        val pref = applicationContext.getSharedPreferences("token", Context.MODE_PRIVATE)
//        val token = pref.getString("token", null)

//        val requestBody = mapOf(
//            "storeId" to storeId,
//            "roomNumber" to roomNum,
//            "fcmToken" to token
//        )

        client.sendStoreInfo(storeId, roomNum).enqueue(object : Callback<Int> {
            override fun onResponse(call : Call<Int>, response: Response<Int>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
//                        Log.d("Login", "login success${response}")
                        Log.d("Login", "login success body ${responseBody}")
                        val roomId = responseBody
                        saveStoreInfo(storeId, roomId)
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        ShowDialog.showFailure(this@InitialActivity, "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    ShowDialog.showFailure(this@InitialActivity, "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<Int>, t: Throwable) {
                Log.e("Login", "$t")
                ShowDialog.showFailure(this@InitialActivity, "요청에 실패했습니다.")
            }
        })
    }

    private fun saveStoreInfo(storeId: String, roomId: Int) {
        val sharedPreferences = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("storeId", storeId)
        editor.putString("roomId", roomId.toString())
        editor.apply()

        Log.d("Login", "저장 성공 룸 아이디: ${roomId}")
        val intent = Intent(this, MainActivity::class.java)
        startActivity(intent)
        finish()
    }
}