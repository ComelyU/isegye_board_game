package com.example.isegyeboard.room

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.navigation.findNavController
import androidx.navigation.fragment.findNavController
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.FailureDialog
import com.example.isegyeboard.game_detail.GameDetail
import com.example.isegyeboard.game_detail.GameOrderApi
import com.example.isegyeboard.main_page.Tutorial
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class StartFragment : Fragment() {
    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.fragment_start, container, false)

        val peopleCountText = view.findViewById<TextView>(R.id.peopleCount)
        var peopleCount = 2
        val plusButton = view.findViewById<TextView>(R.id.peoplePlusButton)
        val minusButton = view.findViewById<TextView>(R.id.peopleMinusButton)

        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val isOccupied = sharedPreferences.getString("isOccupied", null)

        if (isOccupied != null) {
            findNavController().navigate(R.id.action_startFragment_to_main_page_frg)
        }

        peopleCountText.text = peopleCount.toString()

        plusButton.setOnClickListener {
            // 인원수 증가
            peopleCount++
            // 최소값 제한
            if (peopleCount > 10) { // 최대값 설정 예시
                peopleCount = 10
            }
            peopleCountText.text = peopleCount.toString()
        }

        minusButton.setOnClickListener {
            // 인원수 감소
            peopleCount--
            // 최소값 제한
            if (peopleCount < 1) {
                peopleCount = 1
            }
            peopleCountText.text = peopleCount.toString()
        }

        view.findViewById<ConstraintLayout>(R.id.startButton).setOnClickListener{
            // api 설계되면 밑에 링크는 주석
//            findNavController().navigate(R.id.action_startFragment_to_main_page_frg)
            // 윗 줄 주석
            showStartDialog(peopleCount)
        }

        view.findViewById<ConstraintLayout>(R.id.startGuideButton).setOnClickListener {
            val intent = Intent(requireContext(), Tutorial::class.java)
            startActivity(intent)
        }

        view.findViewById<TextView>(R.id.logoutButton).setOnClickListener{
            showLogoutDialog()
        }

        return view
    }

    private fun showStartDialog(peopleCount: Int) {
        val alertDialogBuilder = AlertDialog.Builder(requireActivity())
        alertDialogBuilder.apply {
            setTitle("인원을 확인해 주세요!")
            setMessage("플레이어 수가 $peopleCount 명이 맞으신가요?")
            setPositiveButton("맞아요!") { _, _ ->
                sendStartInfo(peopleCount)
            }
            setNegativeButton("아니에요.") { dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun sendStartInfo(peopleNum: Int) {
        val client = BaseApi.getInstance().create(RoomApi::class.java)

        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val roomId = sharedPreferences.getString("roomId", "1")

        val requestBody = mapOf(
            "isTheme" to 1,
            "peopleNum" to peopleNum,
        )

        client.sendRoomInfo(roomId!!, requestBody).enqueue(object : Callback<RoomStartResponse> {
            override fun onResponse(call : Call<RoomStartResponse>, response: Response<RoomStartResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        roomOccupy(responseBody.id)
                        Log.d("Start", "Start success")
                    } else {
                        Log.d("Start", "Start failed $responseBody")
                        FailureDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Start", "request failed $response")
                    FailureDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<RoomStartResponse>, t: Throwable) {
                Log.e("Start", "$t")
                FailureDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
    }

    private fun roomOccupy(customerId: Int) {
        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("isOccupied", "1")
        editor.putBoolean("theme", true)
        editor.putInt("volume", 100)
        editor.putString("customerId", customerId.toString())
        editor.apply()
        Log.d("Start", "저장 성공 customerId : ${customerId}")

        sendVolume(customerId.toString(), 100)
        findNavController().navigate(R.id.action_startFragment_to_main_page_frg)
    }

    private fun showLogoutDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireActivity())
        alertDialogBuilder.apply {
            setTitle("연결해제")
            setMessage("연결을 해제하시겠습니까?\n일반손님은 취소 버튼을 눌러 주세요!")
            setPositiveButton("종료") { _, _ ->
                requireView().findNavController().navigate(R.id.action_startFragment_to_logoutFragment)
            }
            setNegativeButton("취소") { dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun sendVolume(customerId: String, volume: Int){

        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        service.sendVolume(customerId, volume.toString()).enqueue(object : Callback<Void> {
            override fun onResponse(call : Call<Void>, response: Response<Void>) {
                if (response.isSuccessful) {
                    Log.d("Volume", "Success : $volume")
                } else {
                    Log.d("Volume", "Fail")
                }
            }
            override fun onFailure(call: Call<Void>, t:Throwable) {
                Log.d("Volume", "$t")
            }
        })
    }
}