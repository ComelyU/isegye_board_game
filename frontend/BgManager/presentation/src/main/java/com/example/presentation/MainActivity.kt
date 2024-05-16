package com.example.presentation

import android.content.Context
import android.os.Bundle
import androidx.activity.viewModels
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.Observer
import com.example.presentation.adapter.GameAdapter
import com.example.presentation.adapter.OrderAdapter
import com.example.presentation.adapter.TurtleAdapter
import com.example.presentation.databinding.ActivityMainBinding
import com.example.presentation.ui.UiState
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity :
    AppCompatActivity(),
    TurtleAdapter.TurtleOnClickListener,
    GameAdapter.GameOnClickListener,
    OrderAdapter.OrderOnClickListener
{

    private val viewModel: MainViewModel by viewModels()

    private var _binding: ActivityMainBinding? = null
    private lateinit var orderAdapter: OrderAdapter
    private lateinit var gameAdapter: GameAdapter
    private lateinit var turtleAdapter: TurtleAdapter

    private val binding
        get() = _binding!!
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
//        enableEdgeToEdge()
        _binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

//        if (!isLogined()) {
//            val intent = Intent(this, LoginActivity::class.java)
//            startActivity(intent)
//        }

        binding.viewModel = viewModel
        binding.lifecycleOwner = this

        orderAdapter = OrderAdapter(this)
        gameAdapter = GameAdapter(this)
        turtleAdapter = TurtleAdapter(this)

        binding.mainRV.adapter = orderAdapter
        binding.gameRV.adapter = gameAdapter
        binding.turtleRV.adapter = turtleAdapter

        viewModel.loadData()

        viewModel.uiStateFlow.observe(this, Observer { uiState ->
            when (uiState) {
                is UiState -> {
                    orderAdapter.submitList(uiState.orders)
                    gameAdapter.submitList(uiState.games)
                    turtleAdapter.submitList(uiState.turtles)
                }
                // 다른 상태 처리
            }
        })
    }

    private fun isLogined() : Boolean {
        val sharedPreferences = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        return sharedPreferences.contains("roomId")
    }

    override fun onTurtleClicked(turtleId: Int) {
        // 여기서 selectTurtle 함수를 호출합니다.
        viewModel.selectTurtle(turtleId)
        showAlert("${turtleId}번 로봇이 선택되었습니다.")
    }

    override fun onGameClicked(gameOrderId: Int, orderType: Int, roomNumber: Int) {
        // 여기서 selectTurtle 함수를 호출합니다.
        viewModel.selectGame(gameOrderId, orderType)
        showAlert("${gameOrderId}번 주문이 담겼습니다.")
    }

    override fun onGameCancelClicked(gameOrderId: Int) {
        viewModel.cancelGameOrder(gameOrderId)
    }

    override fun onOrderClicked(orderId: Int, roomNumber: Int) {
        // 여기서 selectTurtle 함수를 호출합니다.
        viewModel.selectMenuId(orderId)
        showAlert("${orderId}번 주문이 담겼습니다.")
    }

    private fun showAlert(message: String) {
        val builder = AlertDialog.Builder(this)

        builder.setTitle("알림")
        builder.setMessage(message)

        builder.setPositiveButton("확인") {dialog, _ ->
            dialog.dismiss()
        }

        val dialog = builder.create()
        dialog.show()
    }
}