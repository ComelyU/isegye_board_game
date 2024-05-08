package com.example.isegyeboard.game_detail

import android.content.Context
import android.content.SharedPreferences
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.fragment.app.Fragment
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import com.bumptech.glide.Glide
import com.bumptech.glide.request.target.DrawableImageViewTarget
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.BasicResponse
import com.example.isegyeboard.databinding.FragmentGamedetailBinding
import kotlinx.coroutines.launch
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response


class GameDetail : Fragment() {

    private lateinit var binding: FragmentGamedetailBinding
    private lateinit var sharedPreferences: SharedPreferences
    private lateinit var loadingImage: ImageView

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentGamedetailBinding.inflate(inflater, container, false)
        return binding.root

    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        binding.photoButton.setOnClickListener {
            // 테스트용 임시 네비
            it.findNavController().navigate(R.id.action_gamedetail_to_photo)
            // 테스트후 삭제
        }

        loadingImage = view.findViewById<ImageView>(R.id.loadingImage)
        loadingImage.visibility = View.GONE

        Glide.with(this)
            .load(R.drawable.loading)
            .into(DrawableImageViewTarget(loadingImage))

        // 목록으로 돌아가기
        binding.detailBack.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }

        // Argument 에서 클릭된 항목의 정보를 받아와서 UI에 표시
        val gameId = arguments?.getString("gameId")
        val title = arguments?.getString("title")
        val description = arguments?.getString("description")
        val thumbnailUrl = arguments?.getString("thumbnailUrl")
        val minPlayer = arguments?.getString("minPlayer")
        val maxPlayer = arguments?.getString("maxPlayer")
        val minPlaytime = arguments?.getString("minPlaytime")
        val maxPlaytime = arguments?.getString("maxPlaytime")
        val difficulty = arguments?.getString("difficulty")!!.toInt()
        val theme = arguments?.getString("theme")

        // UI 요소에 정보 표시
        binding.detailTitle.text = title
        binding.detailDescription.text = description
        binding.detailPlayer.text = if (minPlayer == maxPlayer) {
            "인원 : ${maxPlayer}명"
        } else {
            "인원 : $minPlayer ~ ${maxPlayer}명"
        }
        binding.detailPlaytime.text = if (minPlaytime == maxPlaytime) {
            "플레이타임 : ${maxPlaytime}분"
        } else {
            "플레이타임 : $minPlaytime ~ ${maxPlaytime}분"
        }
        binding.detailDifficulty.text = "난이도 : ${"★".repeat(difficulty)}"
        binding.detailTheme.text = "장르 : $theme"

        thumbnailUrl?.let {
            Glide.with(requireContext())
                .load(it)
                .placeholder(R.drawable.ipad) // 로딩 중에 표시할 이미지
                .error(R.drawable.chess_black) // 로딩 실패 시 표시할 이미지
                .into(binding.detailImage)
        }
        val customerId = sharedPreferences.getString("customerId", "1")
        val themeOn = sharedPreferences.getBoolean("themeOn", true)
        val savedGameId = sharedPreferences.getString("gameId", "")

        // DB상의 테마 온오프여부
//        println(themeOn)
        binding.themeSwitch.isChecked = themeOn

        // 테마 온오프 스위치
        binding.themeSwitch.setOnCheckedChangeListener { _, _ ->
            themeToggle(customerId!!)
        }

        // 시작/반납 버튼 토글
        toggleButtons(gameId.toString(), savedGameId, view)

        // 시작버튼
        binding.startButton.setOnClickListener{
            lifecycleScope.launch {
                if (savedGameId != null) {
                    //모달
                    loadingImage.visibility = View.VISIBLE
                    orderGame(savedGameId, customerId!!, view, 1)
                    gameId?.let{orderGame(it, customerId, view, 0)}
                } else {
                    loadingImage.visibility = View.VISIBLE
                    gameId?.let { orderGame(it, customerId!!, view, 0) }
                }
            }
        }

        //종료버튼
        binding.returnButton.setOnClickListener{
            lifecycleScope.launch {
                loadingImage.visibility = View.VISIBLE
                gameId?.let{orderGame(it, customerId!!, view, 1)}
            }
        }
    }



    private fun orderGame(gameId: String, customerId: String, view: View, orderType: Int) {
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        val requestBody = mapOf(
            "orderType" to orderType,
        )

        val call: Call<GameOrderResponse> = service.orderGame(gameId, customerId, requestBody)
        Log.d("GameOrder", "$gameId, $customerId, $requestBody")

        call.enqueue(object : Callback<GameOrderResponse> {
            override  fun onResponse(call: Call<GameOrderResponse>, response: Response<GameOrderResponse>) {
                loadingImage.visibility = View.GONE
                if (response.isSuccessful) {
                    val responseBody = response.body()

                    // 성공 여부에 따른 로직 처리
                    if (responseBody != null && orderType == 0) {
                        Log.d("Order", "Success")
                        saveGameId(gameId, view)
                    } else if (responseBody != null && orderType == 1) {
                        sharedPreferences.edit().remove("gameId").apply()
                        showOrderDialog("반납")
                        toggleButtons(gameId, null, view)
                    } else {
                        // 실패 처리
                        showFailedDialog()
                        Log.d("Order", "Failed but response success")
                    }
                } else {
                    showFailedDialog()
                    Log.d("Order", "Response fail: ${response.errorBody()}")
                }
            }
            override fun onFailure(call: Call<GameOrderResponse>, t:Throwable) {
                showFailedDialog()
                Log.d("Order", "Fail to send : $t")
            }
        })
    }

    private fun saveGameId(gameId: String, view: View) {
        val sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("gameId", gameId)
//        editor.putString("isDeli", "true")
        editor.apply()
        showOrderDialog("주문")
        toggleButtons(gameId, gameId, view)
    }

    private fun showOrderDialog(message: String) {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("요청완료")
            setMessage("${message}요청되었습니다.\n잠시만 기다려주세요")
            setPositiveButton("확인") {dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun showFailedDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("요청실패")
            setMessage("네트워크 오류가 발생했습니다.\n잠시 후 다시 시도해주세요.")
            setPositiveButton("확인") {dialog, _ ->
                loadingImage.visibility = View.GONE
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun toggleButtons(gameId: String, savedGameId: String?, view: View) {
        val returnButton = view.findViewById<TextView>(R.id.returnButton)
        val startButton = view.findViewById<TextView>(R.id.startButton)

        if (gameId == savedGameId) {
            returnButton.visibility = View.VISIBLE
            startButton.visibility = View.GONE
        } else {
            returnButton.visibility = View.GONE
            startButton.visibility = View.VISIBLE
        }
    }

    private fun themeToggle(customerId: String){
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        val requestBody = mapOf(
            "customerId" to customerId
        )

        service.sendThemeToggle(requestBody).enqueue(object : Callback<BasicResponse> {
            override fun onResponse(call : Call<BasicResponse>, response: Response<BasicResponse>) {
                if (response.isSuccessful) {
                    Log.d("ThemeToggle", "Success : $response")
                } else {
                    Log.d("ThemeToggle", "$response")
                }
            }
            override fun onFailure(call: Call<BasicResponse>, t:Throwable) {
                Log.d("ThemeToggle", "$t")
            }
        })
    }
}