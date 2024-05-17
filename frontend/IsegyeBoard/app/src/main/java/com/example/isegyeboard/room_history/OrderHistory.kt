package com.example.isegyeboard.room_history

import android.content.Context
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.isegyeboard.databinding.FragmentOrderHistoryBinding
import com.example.isegyeboard.room_history.model.OrderGameResponse
import com.example.isegyeboard.room_history.model.OrderMenuResponse
import kotlinx.coroutines.launch

class OrderHistory : Fragment() {
    private lateinit var binding: FragmentOrderHistoryBinding

    private lateinit var viewModel: HistoryViewModel

    private lateinit var menuHistoryAdapter: MenuHistoryAdapter
    private lateinit var menuHistoryList: List<OrderMenuResponse>

    private lateinit var gameHistoryAdapter: GameHistoryAdapter
    private lateinit var gameHistoryList: List<OrderGameResponse>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        // Inflate the layout for this fragment
        binding = FragmentOrderHistoryBinding.inflate(layoutInflater)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        val sharedPreferences =
            requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val customerId = sharedPreferences.getString("customerId", "1")

        viewModel = HistoryViewModel(customerId!!)
        viewModel.getMenuHistoryList()
        viewModel.getGameHistoryList()

        menuHistoryAdapter = MenuHistoryAdapter(requireContext(), emptyList()) // 초기에 빈 리스트로 어댑터 생성
        binding.menuHistoryRV.adapter = menuHistoryAdapter
        binding.menuHistoryRV.layoutManager = LinearLayoutManager(requireContext())

        gameHistoryAdapter = GameHistoryAdapter(requireContext(), emptyList()) // 초기에 빈 리스트로 어댑터 생성
        binding.gameHistoryRV.adapter = gameHistoryAdapter
        binding.gameHistoryRV.layoutManager = LinearLayoutManager(requireContext())

        val emptyTextView = binding.emptyTextView

        lifecycleScope.launch {
            viewModel.menuHistoryList.observe(viewLifecycleOwner) { historylist ->
//                println(historylist)

                if (historylist.isNotEmpty()) {
                    emptyTextView.visibility = View.GONE // 비어 있지 않은 경우 TextView 숨김
                    menuHistoryAdapter.updateData(historylist) // 데이터 업데이트
                    menuHistoryList = historylist
                }
            }

            viewModel.gameHistoryList.observe(viewLifecycleOwner) { historylist ->
//                println(historylist)

                if (historylist.isNotEmpty()) {
                    emptyTextView.visibility = View.GONE // 비어 있지 않은 경우 TextView 숨김
                    gameHistoryAdapter.updateData(historylist) // 데이터 업데이트
                    gameHistoryList = historylist
                }
            }
        }
    }
}
