package com.example.isegyeboard.room_history

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.navigation.fragment.findNavController
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.FailureDialog
import com.example.isegyeboard.login.StartApi
import com.example.isegyeboard.main_page.Tutorial
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class StartFragment : Fragment() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val isOcupied = sharedPreferences.getString("isOcupied", null)

        if (isOcupied != null) {
            findNavController().navigate(R.id.action_startFragment_to_main_page_frg)
        }

    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.fragment_start, container, false)

        view.findViewById<ConstraintLayout>(R.id.startButton).setOnClickListener{
            // api 설계되면 밑에 링크는 주석
            findNavController().navigate(R.id.action_startFragment_to_main_page_frg)
            // 윗 줄 주석
            sendStartInfo()
        }

        view.findViewById<ConstraintLayout>(R.id.startGuideButton).setOnClickListener {
            val intent = Intent(requireContext(), Tutorial::class.java)
            startActivity(intent)
        }

        return view
    }

    private fun sendStartInfo() {
        val client = BaseApi.getInstance().create(StartApi::class.java)

        val sharedPreferences = requireContext().getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val storeId = sharedPreferences.getString("storeId", null)
        val roomNum = sharedPreferences.getString("roomNum", null)

        val requestBody = mapOf(
            "storeId" to storeId,
            "roomNumber" to roomNum,
        )

        client.sendRoomInfo(requestBody).enqueue(object : Callback<RoomStartResponse> {
            override fun onResponse(call : Call<RoomStartResponse>, response: Response<RoomStartResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {
                        Occupy(responseBody.roomLogId)
                        Log.d("Login", "login success")
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        FailureDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    FailureDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<RoomStartResponse>, t: Throwable) {
                Log.e("Login", "$t")
                FailureDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
    }

    private fun Occupy(roomLogId: Int) {
        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("isOccupied", "1")
        editor.putString("roomLogId", roomLogId.toString())
        editor.apply()
        findNavController().navigate(R.id.action_startFragment_to_main_page_frg)
    }
}