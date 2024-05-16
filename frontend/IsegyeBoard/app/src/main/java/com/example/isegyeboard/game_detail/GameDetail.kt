package com.example.isegyeboard.game_detail

import android.content.Context
import android.content.SharedPreferences
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.fragment.app.Fragment
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import com.bumptech.glide.Glide
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.ShowDialog
import com.example.isegyeboard.databinding.FragmentGamedetailBinding
import kotlinx.coroutines.launch
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response


class GameDetail : Fragment() {

    private lateinit var binding: FragmentGamedetailBinding
    private lateinit var sharedPreferences: SharedPreferences
    private var stock : String = "0"
    private var savedOrderId : String = "0"

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
            it.findNavController().navigate(R.id.action_gamedetail_to_photo)
        }

//        Glide.with(this)
//            .load(R.drawable.loading)
//            .into(DrawableImageViewTarget(loadingImage))

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
        stock = arguments?.getString("stock")!!

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
        val volume = sharedPreferences.getInt("volume", 100)
        val savedGameId = sharedPreferences.getString("gameId", "")
        savedOrderId = sharedPreferences.getString("isDeli", "")!!

        binding.themeSwitch.isChecked = themeOn
//        println(themeOn)

        // 테마 온오프 스위치
        binding.themeSwitch.setOnCheckedChangeListener { _, _ ->
            sendThemeToggle(customerId!!)
        }

        val seekBar = binding.volumeSeekBar
        seekBar.progress = volume
        Log.d("Volume", "savedVolume : $volume")

        seekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                // 사용자에 의해 변경된 경우에만 SharedPreferences에 저장합니다.
                if (fromUser) {
                    sharedPreferences.edit().putInt("volume", progress).apply()
                }
            }

            override fun onStartTrackingTouch(seekBar: SeekBar?) {
                // 사용자가 SeekBar를 터치할 때 호출됩니다.
            }

            override fun onStopTrackingTouch(seekBar: SeekBar?) {
                // 사용자가 SeekBar 터치를 멈출 때 호출됩니다.
                val currentProgress = seekBar!!.progress
                sendVolume(customerId!!, currentProgress)
            }
        })

        // 시작/반납 버튼 토글
        orderButtonToggle(gameId.toString(), savedGameId, view)

        // 시작버튼
        if (stock != "0") {
            binding.startButton.setOnClickListener{
                lifecycleScope.launch {
                    if (savedGameId != "") {
//                        val check = checkOrder(savedOrderId!!)
//                        showReturnOrderDialog(savedGameId, gameId.toString(), customerId!!, view)
                        orderCollection(gameId.toString(), savedGameId, savedOrderId, customerId!!, view, 2)
                    } else {
//                        gameId?.let { orderGame(it, customerId!!, view, 0) }
                        orderCollection(gameId.toString(), null, null, customerId!!, view, 0)
                    }
                }
            }
        } else {
            binding.startButton.setBackgroundResource(R.drawable.grey_rad)
            binding.startButton.setOnClickListener{
                lifecycleScope.launch {
                    ShowDialog.showFailure(requireContext(), "해당 보드게임의 재고가 없습니다.")
                }
            }
        }

        //종료버튼
        binding.returnButton.setOnClickListener{
            lifecycleScope.launch {
                orderCollection(gameId.toString(), savedGameId, savedOrderId, customerId!!, view, 1)
            }
        }
    }

    private fun showOrderDialog(message: String) {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("주문완료")
            setMessage(message)
            setPositiveButton("확인") {dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun orderButtonToggle(gameId: String, savedGameId: String?, view: View) {
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

    private fun sendThemeToggle(customerId: String){

        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        service.sendThemeToggle(customerId).enqueue(object : Callback<Int> {
            override fun onResponse(call : Call<Int>, response: Response<Int>) {
                if (response.isSuccessful) {
                    Log.d("ThemeToggle", "Success : ${response.body()}")
                    val editor = sharedPreferences.edit()

                    if (response.body() == 1) {
                        editor.putBoolean("themeOn", true).apply()
                    } else {
                        editor.putBoolean("themeOn", false).apply()
                    }
                } else {
                    Log.d("ThemeToggle", "Fail: $response")
                }
            }
            override fun onFailure(call: Call<Int>, t:Throwable) {
                Log.d("ThemeToggle", "$t")
            }
        })
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



    private fun orderCollection(
        gameId: String, savedGameId: String?, orderId: String?, customerId: String, view: View, orderType: Int
    ) {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())

        // 일반 주문 요청
        if (orderType == 0) {
            gameOrder(gameId, customerId) { result ->
                if (result != 0.toLong()) {
                    saveGameId(gameId, result.toString(), view)
//                    println(gameId)
                } else {
                    showOrderDialog("에러")
                }
                showOrderDialog("주문이 완료되었습니다.")
            }

        // 일반 반납 요청
        } else if (orderType == 1) {
            println("orderId $savedOrderId")
            checkOrder(savedOrderId) { orderState ->
                when (orderState) {
                    0 -> { // 취소 후 주문
                        alertDialogBuilder.apply {
                            setTitle("알림")
                            setMessage("이미 진행중인 주문이 있습니다.\n기존 주문을 취소하시겠습니까?")
                            setPositiveButton("확인") {dialog, _ ->
                                cancelOrder(savedOrderId) {result ->
                                    if (result == 1) {
                                        removeGameId(gameId, view)
                                        showOrderDialog("주문이 취소되었습니다.")
                                    }
                                }
                                dialog.dismiss()
                            }
                            setNegativeButton("취소") {dialog, _ ->
                                dialog.dismiss()
                            }
                        }
                        val alertDialog = alertDialogBuilder.create()
                        alertDialog.show()
                    }
                    1 -> {// 배달중 취소 및 반납 불가
                        ShowDialog.showFailure(requireContext(), "현재 배달 중인 게임이 있습니다.\n배달 완료 후 다시 요청해 주세요.")
                    }
                    2 -> {// 배달 완료 -> 반납 후 주문 요청
                        returnOrder(gameId, customerId) { result ->
                            if (result == 1) {
                                removeGameId(gameId, view)
                                showOrderDialog("반납요청되었습니다.")
                            }
                        }
                    }
                }
            }

        // 반납 + 주문 요청
        } else if (orderType == 2 && savedGameId != null) {
            checkOrder(orderId!!) { orderState ->
                when (orderState) {
                    0 -> {
                        alertDialogBuilder.apply {
                            setTitle("알림")
                            setMessage("이미 진행중인 주문이 있습니다.\n" +
                                    "기존 주문을 취소하고 새 게임을 대여하시겠습니까?")
                            setPositiveButton("확인") {dialog, _ ->
                                cancelOrder(orderId) {result ->
                                    if (result == 1) {
                                        removeGameId(savedGameId, view)
                                    } else {
                                        showOrderDialog("취소 에러")
                                    }
                                }
                                gameOrder(gameId, customerId) { result ->
                                    if (result != 0.toLong()) {
                                        saveGameId(gameId, result.toString(), view)
                                    } else {
                                        showOrderDialog("주문 에러")
                                    }
                                }
                                dialog.dismiss()
                                showOrderDialog("취소 및 주문이 완료되었습니다.")
                            }
                            setNegativeButton("취소") {dialog, _ ->
                                dialog.dismiss()
                            }
                        }
                        val alertDialog = alertDialogBuilder.create()
                        alertDialog.show()

                        // 배달중 취소 및 반납 불가
                    }
                    1 -> {
                        ShowDialog.showFailure(requireContext(), "현재 배달 중인 게임이 있습니다.\n배달 완료 후 다시 요청해 주세요.")

                        // 배달 완료 -> 반납 후 주문 요청
                    }
                    2 -> {
                        alertDialogBuilder.apply {
                            setTitle("알림")
                            setMessage("이미 대여중인 보드게임이 있습니다.\n기존 보드게임을 반납하고 새 게임을 대여하시겠습니까?")
                            setPositiveButton("확인") {dialog, _ ->
                                returnOrder(savedGameId, customerId) { result ->
                                    if (result == 1) {
                                        removeGameId(savedGameId, view)
                                    } else {
                                        showOrderDialog("반납 에러")
                                    }
                                }
                                gameOrder(gameId, customerId) { result ->
                                    if (result != 0.toLong()) {
                                        saveGameId(gameId, result.toString(), view)
                                    } else {
                                        showOrderDialog("주문 에러")
                                    }
                                }
                                dialog.dismiss()
                                showOrderDialog("반납 및 주문이 완료되었습니다.")
                            }
                            setNegativeButton("취소") {dialog, _ ->
                                dialog.dismiss()
                            }
                        }
                        val alertDialog = alertDialogBuilder.create()
                        alertDialog.show()

                        // 취소된 주문
                    }
                    3 -> {
                        removeGameId(savedGameId, view)
                        gameOrder(gameId, customerId) { result ->
                            if (result != 0.toLong()) {
                                saveGameId(gameId, result.toString(), view)
                            } else {
                                showOrderDialog("주문 에러")
                            }
                        }
                        showOrderDialog("주문이 완료되었습니다.")

                        // 통신 오류
                    }
                    256 -> {
                        ShowDialog.showFailure(requireContext(), "통신에 오류가 발생했습니다.")

                        // 배송오류 발생 -> 취소후 주문
                    }
                    else -> {
                        alertDialogBuilder.apply {
                            setTitle("알림")
                            setMessage("배송 중 오류가 확인돼었습니다.\n기존 주문을 취소하고 새 게임을 대여합니다.")
                            setPositiveButton("확인") {dialog, _ ->
                                cancelOrder(orderId) {result ->
                                    if (result == 1) {
                                        removeGameId(savedGameId, view)
                                    } else {
                                        showOrderDialog("취소 에러")
                                    }
                                }
                                gameOrder(gameId, customerId) { result ->
                                    if (result != 0.toLong()) {
                                        saveGameId(gameId, result.toString(), view)
                                    } else {
                                        showOrderDialog("주문 에러")
                                    }
                                }
                                dialog.dismiss()
                                showOrderDialog("취소 및 주문이 완료되었습니다.")
                            }
                            setNegativeButton("취소") {dialog, _ ->
                                dialog.dismiss()
                            }
                        }
                        val alertDialog = alertDialogBuilder.create()
                        alertDialog.show()
                    }
                }
            }
        } else {
            ShowDialog.showFailure(requireContext(), "잘못된 요청 입니다.")
        }
    }

    private fun gameOrder(gameId: String, customerId: String, callback: (Long?) -> Unit) {
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        val requestBody = mapOf(
            "orderType" to 0,
        )

        val call: Call<GameOrderResponse> = service.orderGame(gameId, customerId, requestBody)
        Log.d("GameOrder", "$gameId, $customerId, $requestBody")

        call.enqueue(object : Callback<GameOrderResponse> {
            override fun onResponse(call: Call<GameOrderResponse>, response: Response<GameOrderResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        val newOrderId = responseBody.id
                        callback(newOrderId)
                        Log.d("Order", "Success, $newOrderId")
                    } else {
                        callback(null)
                        Log.d("Order", "Failed but response success")
                    }
                } else {
                    callback(null)
                    Log.d("Order", "Response fail: ${response.errorBody()}")
                }
            }
            override fun onFailure(call: Call<GameOrderResponse>, t:Throwable) {
                callback(null)
                Log.d("Order", "Fail to send : $t")
            }
        })
    }

    private fun returnOrder(gameId: String, customerId: String, callback: (Int?) -> Unit) {
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        val requestBody = mapOf(
            "orderType" to 1,
        )

        val call: Call<GameOrderResponse> = service.orderGame(gameId, customerId, requestBody)
        Log.d("GameOrder", "$gameId, $customerId, $requestBody")

        call.enqueue(object : Callback<GameOrderResponse> {
            override fun onResponse(call: Call<GameOrderResponse>, response: Response<GameOrderResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        callback(1)
                        stock = "1"
                        Log.d("Order", "Return Success")
                    } else {
                        callback(null)
                        Log.d("Order", "Failed but response success")
                    }
                } else {
                    callback(null)
                    Log.d("Order", "Response fail: ${response.errorBody()}")
                }
            }
            override fun onFailure(call: Call<GameOrderResponse>, t:Throwable) {
                callback(null)
                Log.d("Order", "Fail to send : $t")
            }
        })
    }

    private fun checkOrder(orderId: String, callback: (Int?) -> Unit) {
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        service.checkOrder(orderId).enqueue(object : Callback<GameOrderResponse> {
            override fun onResponse(call : Call<GameOrderResponse>, response: Response<GameOrderResponse>) {
                if (response.isSuccessful) {
                    Log.d("checkGameOrder", "Check Success : ${response.body()?.orderStatus}")
                    callback(response.body()?.orderStatus)
                } else {
                    Log.d("GameOrder", "Check Fail")
                    callback(null)
                }
            }
            override fun onFailure(call: Call<GameOrderResponse>, t:Throwable) {
                Log.d("GameOrder", "$t")
                callback(null)
            }
        })
    }

    private fun cancelOrder(orderId: String, callback: (Int?) -> Unit){
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))
        Log.d("CancelOrder", "order id : $orderId")
        service.cancelOrder(orderId).enqueue(object : Callback<Void> {
            override fun onResponse(call : Call<Void>, response: Response<Void>) {
                if (response.isSuccessful) {
                    Log.d("CancelOrder", "Cancel Success : $response")
                    callback(1)
                    stock = "1"
                } else {
                    Log.d("CancelOrder", "Cancel Fail")
                    callback(400)
                }
            }
            override fun onFailure(call: Call<Void>, t:Throwable) {
                Log.d("CancelOrder", "$t")
                callback(401)
            }
        })
    }

    private fun saveGameId(gameId: String, orderId:String, view: View) {
        val editor = sharedPreferences.edit()
        editor.putString("gameId", gameId)
        editor.putString("isDeli", orderId)
        editor.apply()
        savedOrderId = orderId
//        println("saveId : $savedOrderId, orderId: $orderId")
        orderButtonToggle(gameId, gameId, view)
    }

    private fun removeGameId(gameId: String, view: View) {
        val editor = sharedPreferences.edit()
        editor.remove("gameId")
        editor.remove("isDeli")
        editor.apply()
        orderButtonToggle(gameId, null, view)
    }


//    private fun orderGame(gameId: String, customerId: String, view: View, orderType: Int) {
//        val service = BaseApi.getInstance().create((GameOrderApi::class.java))
//
//        val requestBody = mapOf(
//            "orderType" to orderType,
//        )
//
//        val call: Call<GameOrderResponse> = service.orderGame(gameId, customerId, requestBody)
//        Log.d("GameOrder", "$gameId, $customerId, $requestBody")
//
//        call.enqueue(object : Callback<GameOrderResponse> {
//            override  fun onResponse(call: Call<GameOrderResponse>, response: Response<GameOrderResponse>) {
//                if (response.isSuccessful) {
//                    val responseBody = response.body()
//
//                    // 성공 여부에 따른 로직 처리
//                    if (responseBody != null && orderType == 0) {
//                        Log.d("Order", "Success")
//                        saveGameId(gameId, responseBody.id.toString(), view)
//                    } else if (responseBody != null && orderType == 1) {
//                        sharedPreferences.edit().remove("gameId").apply()
//                        showOrderDialog("반납")
//                        orderButtonToggle(gameId, null, view)
//                    } else {
//                        // 실패 처리
//                        ShowDialog.showFailure(requireContext(), "주문에 오류가 발생했습니다.")
//                        Log.d("Order", "Failed but response success")
//                    }
//                } else {
//                    ShowDialog.showFailure(requireContext(), "서버에 오류가 발생했습니다.")
//                    Log.d("Order", "Response fail: ${response.errorBody()}")
//                }
//            }
//            override fun onFailure(call: Call<GameOrderResponse>, t:Throwable) {
//                ShowDialog.showFailure(requireContext(), "주문요청에 실패했습니다.")
//                Log.d("Order", "Fail to send : $t")
//            }
//        })
//    }
}