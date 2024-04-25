package com.example.isegyeboard.main_page

import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.net.Uri
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.MediaController
import android.widget.VideoView
import androidx.appcompat.app.AlertDialog
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.navigation.findNavController
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.BasicResponse
import com.example.isegyeboard.baseapi.FailureDialog
import com.example.isegyeboard.login.StartApi
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class MainPageFrg : Fragment() {

    private lateinit var sharedPreferences: SharedPreferences

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_main_page_frg, container, false)

        val videoView: VideoView = view.findViewById(R.id.homeVideo)
        val mediaController = MediaController(requireContext())
        mediaController.setAnchorView(videoView)
        videoView.setMediaController(mediaController)

        val videoPath = "android.resource://" + requireContext().packageName + "/" + R.raw.scrabble
        val videoUri = Uri.parse(videoPath)
        videoView.setVideoURI(videoUri)
        videoView.setOnPreparedListener { mp -> mp.isLooping = true }

        videoView.start()

        view.findViewById<ConstraintLayout>(R.id.toGamelistButton).setOnClickListener {
            it.findNavController().navigate(R.id.action_main_page_frg_to_gamelist)
        }

        view.findViewById<ConstraintLayout>(R.id.toRecommendButton).setOnClickListener {
            it.findNavController().navigate(R.id.action_main_page_frg_to_recommend)
        }

        view.findViewById<ConstraintLayout>(R.id.toBeverage).setOnClickListener {
            it.findNavController().navigate(R.id.action_main_page_frg_to_beverage)
        }

        view.findViewById<ConstraintLayout>(R.id.homeTutorialButton).setOnClickListener {

        }

        view.findViewById<ConstraintLayout>(R.id.videoSpace).setOnClickListener {

        }

        view.findViewById<ConstraintLayout>(R.id.button6).setOnClickListener {
            it.findNavController().navigate(R.id.action_main_page_frg_to_order_history)
        }
        view.findViewById<ConstraintLayout>(R.id.button7).setOnClickListener {
            showOverDialog()
        }
        return view
    }

    private fun showOverDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireActivity())
        alertDialogBuilder.apply {
            setTitle("사용종료")
            setMessage("사용을 종료하시겠습니까?")
            setPositiveButton("종료") { _, _ ->
                overConfirm()
            }
            setNegativeButton("취소") { dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun overConfirm() {
        val client = BaseApi.getInstance().create(StartApi::class.java)

        val sharedPreferences = requireContext().getSharedPreferences(
            "RoomInfo",
            Context.MODE_PRIVATE
        )
        val pref = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        sharedPreferences.edit().remove("isOccupied").apply()

        val roomLogId = pref.getString("roomLogId", null)

        val requestBody = mapOf(
            "roomLogId" to roomLogId
        )

        client.deleteRoomInfo(requestBody).enqueue(object : Callback<BasicResponse> {
            override fun onResponse(call: Call<BasicResponse>, response: Response<BasicResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {

                        sharedPreferences.edit().remove("isOccupied").apply()

                        //페이지 이동
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

            override fun onFailure(call: Call<BasicResponse>, t: Throwable) {
                Log.e("Login", "$t")
                FailureDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
    }
}