package com.example.presentation

import android.os.Bundle
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.Observer
import com.example.presentation.adapter.OrderAdapter
import com.example.presentation.databinding.ActivityMainBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : AppCompatActivity() {

    private val viewModel: MainViewModel by viewModels()
    private lateinit var adapter: OrderAdapter

    private var _binding: ActivityMainBinding? = null

    private val binding
        get() = _binding!!
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
//        enableEdgeToEdge()
        _binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.viewModel = viewModel
        binding.lifecycleOwner = this

        val adapter = OrderAdapter(
//            listOf(
//                OrderUiState(orderId = 5, orderQuantity = 5, orderName = "메뉴1"),
//                OrderUiState(orderId = 6, orderQuantity = 6, orderName = "메뉴2"),
//                OrderUiState(orderId = 7, orderQuantity = 7, orderName = "메뉴3"),
//            )
        )
        binding.mainRV.adapter = adapter

        viewModel.loadData()

        viewModel.uiStateFlow.observe(this, Observer { uiState ->
            adapter.submitList(uiState.orders)
        })
    }
}
//        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
//            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
//            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
//            insets
//        }
//    }
