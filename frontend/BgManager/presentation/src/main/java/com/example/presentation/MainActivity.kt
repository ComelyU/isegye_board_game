package com.example.presentation

import android.os.Bundle
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.Observer
import com.example.presentation.adapter.OrderAdapter
import com.example.presentation.databinding.ActivityMainBinding
import com.example.presentation.ui.UiState
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : AppCompatActivity() {

    private val viewModel: MainViewModel by viewModels()

    private var _binding: ActivityMainBinding? = null
    private lateinit var orderAdapter: OrderAdapter

    private val binding
        get() = _binding!!
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
//        enableEdgeToEdge()
        _binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.viewModel = viewModel
        binding.lifecycleOwner = this

        orderAdapter = OrderAdapter()
        binding.mainRV.adapter = orderAdapter

        viewModel.loadData()

//        viewModel.uiStateFlow.observe(this, Observer { uiState ->
//            adapter.submitList(uiState.orders)
//        })

        viewModel.uiStateFlow.observe(this, Observer { uiState ->
            when (uiState) {
                is UiState -> {
                    orderAdapter.submitList(uiState.orders)
                }
                // 다른 상태 처리
            }
        })
    }
}