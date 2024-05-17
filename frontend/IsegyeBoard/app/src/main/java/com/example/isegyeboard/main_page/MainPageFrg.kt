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
import androidx.navigation.fragment.findNavController
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.ShowDialog
import com.example.isegyeboard.room.RoomApi
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

        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val isOccupied = sharedPreferences.getString("isOccupied", null)
        if (isOccupied == null) {
            findNavController().navigate(R.id.action_main_page_frg_to_startFragment)
        }

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
            val intent = Intent(requireContext(), Tutorial::class.java)
            startActivity(intent)
        }

//        view.findViewById<ConstraintLayout>(R.id.videoSpace).setOnClickListener {
//        }

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
        val client = BaseApi.getInstance().create(RoomApi::class.java)

        val sharedPreferences = requireContext().getSharedPreferences(
            "RoomInfo",
            Context.MODE_PRIVATE
        )
        val pref = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val customerId = pref.getString("customerId", null)
        Log.d("Logout", "try logout : ${customerId}")

        client.deleteCustomerInfo(customerId!!).enqueue(object : Callback<Int> {
            override fun onResponse(call: Call<Int>, response: Response<Int>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        val editor = sharedPreferences.edit()
                        editor.remove("isOccupied")
                        editor.remove("customerId")
                        editor.remove("gameId")
                        editor.remove("idDeli")
                        editor.apply()
                        //페이지 이동
                        Log.d("logout", "logout success")
                        showFeeDialog(responseBody)
                    } else {
                        Log.d("logout", "logout failed $responseBody")
                        ShowDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("logout", "request failed $response")
                    ShowDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<Int>, t: Throwable) {
                Log.e("logout", "$t")
                ShowDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
    }

    private fun showFeeDialog(fee: Int) {
        val alertDialogBuilder = AlertDialog.Builder(requireActivity())
        alertDialogBuilder.apply {
            setTitle("요금 안내")
            setMessage("이용 요금은 ${fee}원 입니다.\n 이용해주셔서 감사합니다.")
            setPositiveButton("종료") { _, _ ->
                findNavController().navigate(R.id.action_main_page_frg_to_startFragment)
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }
}