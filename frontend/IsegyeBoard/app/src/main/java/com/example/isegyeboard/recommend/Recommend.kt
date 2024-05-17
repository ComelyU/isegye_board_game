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
        var selectedTag = ""
        var selectedTime = ""
        var selectedDifficulty = ""

        Glide.with(this)
            .load(R.drawable.loading)
            .into(DrawableImageViewTarget(binding.loadingImageRec))

        binding.themeRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val themeRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTheme = themeRadioButton.tag.toString()
        }
        binding.timeRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val timeRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTime = timeRadioButton.tag.toString()
        }
        binding.tagRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val playerRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedTag = playerRadioButton.tag.toString()
        }
        binding.difficultyRadioGroup.setOnCheckedChangeListener{ _, checkedId ->
            val difficultyRadioButton = view.findViewById<RadioButton>(checkedId)
            selectedDifficulty = difficultyRadioButton.tag.toString()
        }



        binding.searchButton.setOnClickListener{
            binding.loadingImageRec.visibility = View.VISIBLE
            sendRecommendData(selectedTheme, selectedDifficulty, selectedTag, selectedTime)
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

    private fun sendRecommendData(selectedTheme: String, selectedDifficulty:String, selectedTag: String, selectedTime:String) {
        val client = BaseApi.getInstance().create(RecommendApi::class.java)

        client.sendRecommendData(theme = selectedTheme, difficulty = selectedDifficulty, tag = selectedTag, time = selectedTime).enqueue(object : Callback<GameList> {
            override fun onResponse(call : Call<GameList>, response: Response<GameList>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        Log.d("Recommend", "get recommend item success $responseBody")

                        binding.coverImage.visibility = View.GONE

                        binding.reSearchButton.visibility = View.VISIBLE
                        binding.recoStartButton.visibility = View.VISIBLE

//                        gameResponseLiveData.postValue(responseBody)

                    } else {
                        Log.d("Recommend", "Recommend empty $response, ${response.body()}")
                        ShowDialog.showFailure(requireContext(), "추천목록이 비어있습니다.")
                    }
                } else {
                    Log.d("Recommend", "Recommend failed $response")
                    ShowDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<GameList>, t: Throwable) {
                Log.e("Recommend", "$t")
                ShowDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
        binding.loadingImageRec.visibility = View.GONE
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