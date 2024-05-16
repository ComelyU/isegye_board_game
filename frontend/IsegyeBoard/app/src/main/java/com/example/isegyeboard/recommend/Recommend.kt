package com.example.isegyeboard.recommend

import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.RadioButton
import androidx.lifecycle.MutableLiveData
import androidx.navigation.findNavController
import com.bumptech.glide.Glide
import com.bumptech.glide.request.target.DrawableImageViewTarget
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.ShowDialog
import com.example.isegyeboard.databinding.FragmentRecommendBinding
import com.example.isegyeboard.game_list.model.GameResponse
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import kotlin.math.ceil

class Recommend : Fragment() {

    private lateinit var binding : FragmentRecommendBinding
    private val gameResponseLiveData = MutableLiveData<GameResponse?>()

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentRecommendBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        var selectedTheme = ""
        var selectedPlayer = ""
        var selectedTime = ""
        var selecteddifficulty = ""

        Glide.with(this)
            .load(R.drawable.loading)
            .into(DrawableImageViewTarget(binding.loadingImageRec))

        binding.themeRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val themeRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTheme = themeRadioButton.text.toString()
        }
        binding.difficultyRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val difficultyRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedPlayer = difficultyRadioButton.text.toString()
        }
        binding.playerRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val playerRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTime = playerRadioButton.text.toString()
        }
        binding.timeRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val timeRadioButton = view.findViewById<RadioButton>(checkedId)
            selecteddifficulty = timeRadioButton.text.toString()
        }

        val data = RecommendData(selectedTheme, selectedPlayer, selecteddifficulty, selectedTime)

        binding.searchButton.setOnClickListener{
            binding.loadingImageRec.visibility = View.VISIBLE
            sendRecommendData(data)
        }
        binding.recBack.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }
        binding.recoStartButton.setOnClickListener{
            moveToDetail(gameResponseLiveData.value!!)
        }


        gameResponseLiveData.observe(viewLifecycleOwner) { gameResponse ->
            // gameResponse가 변경될 때마다 UI를 업데이트
            binding.recTitle.text = gameResponse?.gameName
            binding.recDescription.text = gameResponse?.gameDetail
            binding.recPlayer.text = if (gameResponse?.minPlayer == gameResponse?.maxPlayer) {
                "인원 : ${gameResponse?.maxPlayer}명"
            } else {
                "인원 : ${gameResponse?.minPlayer} ~ ${gameResponse?.maxPlayer}명"
            }
            binding.recPlaytime.text = if (gameResponse?.minPlaytime == gameResponse?.maxPlaytime) {
                "플레이타임 : ${gameResponse?.maxPlaytime}분"
            } else {
                "플레이타임 : ${gameResponse?.minPlaytime} ~ ${gameResponse?.maxPlaytime}분"
            }
            binding.recDifficulty.text = "난이도 : ${"★".repeat(gameResponse?.gameDifficulty!!.toInt())}"
            val theme = gameResponse.gameTagCategory.joinToString(", ") { category ->
                category.codeItemName
            }
            binding.recTheme.text = "장르 : $theme"
            gameResponse.gameImgUrl.let {
                Glide.with(requireContext())
                    .load(it)
                    .placeholder(R.drawable.ipad) // 로딩 중에 표시할 이미지
                    .error(R.drawable.chess_black) // 로딩 실패 시 표시할 이미지
                    .into(binding.recImage)
            }
        }
    }

    private fun sendRecommendData(data: RecommendData) {
        val client = BaseApi.getInstance().create(RecommendApi::class.java)

        val requestBody = mapOf(
            "theme" to data.selectedTheme,
            "player" to data.selectedTag,
            "difficulty" to data.selectedDifficulty,
            "time" to data.selectedTime
        )

        client.recommendTest("1").enqueue(object : Callback<GameResponse> {
//        client.sendRecommendData(requestBody).enqueue(object : Callback<GameClass> {
            override fun onResponse(call : Call<GameResponse>, response: Response<GameResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        Log.d("Recommend", "get recommend item success")
                        binding.loadingImageRec.visibility = View.GONE
                        binding.coverImage.visibility = View.GONE

                        binding.reSearchButton.visibility = View.VISIBLE
                        binding.recoStartButton.visibility = View.VISIBLE

                        gameResponseLiveData.postValue(responseBody)

                    } else {
                        Log.d("Recommend", "Recommend failed $responseBody")
                        ShowDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Recommend", "Recommend failed $response")
                    ShowDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<GameResponse>, t: Throwable) {
                Log.e("Recommend", "$t")
                ShowDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
    }

    private fun moveToDetail(gameResponse: GameResponse) {
        val bundle = Bundle().apply {
            putString("gameId", gameResponse.id.toString())
            putString("title", gameResponse.gameName)
            putString("description", gameResponse.gameDetail)
            putString("thumbnailUrl", gameResponse.gameImgUrl)
            putString("stock", 1.toString())
            putString("minPlayer", gameResponse.minPlayer.toString())
            putString("maxPlayer", gameResponse.maxPlayer.toString())
            putString("minPlaytime", gameResponse.minPlaytime.toString())
            putString("maxPlaytime", gameResponse.maxPlaytime.toString())
            putString("difficulty", ceil(gameResponse.gameDifficulty).toInt().toString())
            putString("theme", gameResponse.gameTagCategory.joinToString(", ") { category ->
                category.codeItemName
            })
        }
        view?.findNavController()?.navigate(R.id.action_recommend_to_gamedetail, bundle)
    }
}