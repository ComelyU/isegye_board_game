package com.example.presentation

import android.os.Bundle
import androidx.activity.viewModels
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

        binding.viewModel = viewModel
        binding.lifecycleOwner = this

        orderAdapter = OrderAdapter(this)
        gameAdapter = GameAdapter(this)
        turtleAdapter = TurtleAdapter(this)

        binding.mainRV.adapter = orderAdapter
        binding.gameRV.adapter = gameAdapter
        binding.turtleRV.adapter = turtleAdapter

        viewModel.loadData()

//        viewModel.uiStateFlow.observe(this, Observer { uiState ->
//            adapter.submitList(uiState.orders)
//        })

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

    override fun onTurtleClicked(turtleId: Int) {
        // 여기서 selectTurtle 함수를 호출합니다.
        viewModel.selectTurtle(turtleId)
    }

    override fun onGameClicked(gameOrderId: Int, orderType: Int, roomNumber: Int) {
        // 여기서 selectTurtle 함수를 호출합니다.
        viewModel.selectGame(gameOrderId, orderType, roomNumber)
    }

    override fun onOrderClicked(orderId: Int, roomNumber: Int) {
        // 여기서 selectTurtle 함수를 호출합니다.
        viewModel.selectMenuId(orderId, roomNumber)
    }
}