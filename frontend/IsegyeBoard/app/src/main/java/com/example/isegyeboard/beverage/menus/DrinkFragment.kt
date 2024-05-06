package com.example.isegyeboard.beverage.menus

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import androidx.recyclerview.widget.GridLayoutManager
import com.example.isegyeboard.R
import com.example.isegyeboard.beverage.BeverageAdapter
import com.example.isegyeboard.beverage.BeverageClass
import com.example.isegyeboard.beverage.DrinkViewModel
import com.example.isegyeboard.beverage.cart.CartAdapter
import com.example.isegyeboard.databinding.FragmentDrinkBinding
import kotlinx.coroutines.launch

class DrinkFragment : Fragment() {

    private lateinit var binding: FragmentDrinkBinding

    private val viewModel: DrinkViewModel by viewModels()

    private lateinit var cartAdapter: CartAdapter
    private lateinit var beverageAdapter: BeverageAdapter
    private lateinit var noCafeList: List<BeverageClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        // Inflate the layout for this fragment
        binding = FragmentDrinkBinding.inflate(layoutInflater)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        cartAdapter = CartAdapter()
        beverageAdapter = BeverageAdapter(requireContext(), emptyList(), cartAdapter) // 초기에 빈 리스트로 어댑터 생성
        binding.drinkRV.adapter = beverageAdapter
        binding.drinkRV.layoutManager = GridLayoutManager(requireContext(), 4)

        viewModel.getCurrentMenuList()

        lifecycleScope.launch {
            viewModel.drinkMenuList.observe(viewLifecycleOwner) { menuList ->
                beverageAdapter.updateData(menuList) // 데이터 업데이트
                noCafeList = menuList
            }
        }

        // to non-coffee 버튼
        binding.buttonCoffee.setOnClickListener{
            it.findNavController().navigate(R.id.action_drinkFragment_to_coffeFragment)
        }

        // to snack 버튼
        binding.buttonSnack.setOnClickListener{
            it.findNavController().navigate(R.id.action_drinkFragment_to_snackFragment)
        }
    }
}
