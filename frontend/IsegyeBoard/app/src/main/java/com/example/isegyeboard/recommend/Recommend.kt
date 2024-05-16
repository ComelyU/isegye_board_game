package com.example.isegyeboard.recommend

import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.RadioButton
import android.widget.RadioGroup
import android.widget.TextView
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.BasicResponse
import com.example.isegyeboard.baseapi.ShowDialog
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class Recommend : Fragment() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.fragment_recommend, container, false)
        val themeSelect = view.findViewById<RadioGroup>(R.id.themeRadioGroup)
        val difficultySelect = view.findViewById<RadioGroup>(R.id.difficultyRadioGroup)
        val playerSelect = view.findViewById<RadioGroup>(R.id.playerRadioGroup)
        val timeSelect = view.findViewById<RadioGroup>(R.id.timeRadioGroup)

        val searchButton = view.findViewById<TextView>(R.id.searchButton)
        val reSearchButton = view.findViewById<TextView>(R.id.reSearchButton)
        val loadingImage = view.findViewById<ImageView>(R.id.loadingImageRec)
        loadingImage.visibility = View.GONE

        var selectedTheme = ""
        var selectedPlayer = ""
        var selectedTime = ""
        var selecteddifficulty = ""

        themeSelect.setOnCheckedChangeListener{group, checkedId ->
            val themeRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTheme = themeRadioButton.text.toString()
        }
        difficultySelect.setOnCheckedChangeListener{group, checkedId ->
            val difficultyRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedPlayer = difficultyRadioButton.text.toString()
        }
        playerSelect.setOnCheckedChangeListener{group, checkedId ->
            val playerRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTime = playerRadioButton.text.toString()
        }
        timeSelect.setOnCheckedChangeListener{group, checkedId ->
            val timeRadioButton = view.findViewById<RadioButton>(checkedId)
            selecteddifficulty = timeRadioButton.text.toString()
        }

        val data = RecommendData(selectedTheme, selectedPlayer, selecteddifficulty, selectedTime)

        searchButton.setOnClickListener{
            sendRecommendData(data)
        }

        return view
    }

    private fun sendRecommendData(data: RecommendData) {
        val client = BaseApi.getInstance().create(RecommendApi::class.java)

        val requestBody = mapOf(
            "theme" to data.selectedTheme,
            "player" to data.selectedPlayer,
            "difficulty" to data.selecteddifficulty,
            "time" to data.selectedTime
        )

        client.sendRecommendData(requestBody).enqueue(object : Callback<BasicResponse> {
            override fun onResponse(call : Call<BasicResponse>, response: Response<BasicResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {
                        Log.d("Login", "login success")
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        ShowDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    ShowDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<BasicResponse>, t: Throwable) {
                Log.e("Login", "$t")
                ShowDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })

    }
}