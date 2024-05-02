package com.example.isegyeboard.beverage

import android.content.Context
import android.graphics.Color
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R
import com.example.isegyeboard.game_list.GameAdapter
import com.example.isegyeboard.game_list.GameClass
import com.example.isegyeboard.game_list.GameViewModel
import kotlinx.coroutines.launch

class Beverage : Fragment() {
    private lateinit var buttonCoffee: ConstraintLayout
    private lateinit var buttonDrink: ConstraintLayout
    private lateinit var buttonSnack: ConstraintLayout

    private lateinit var textCoffee: TextView
    private lateinit var textDrink: TextView
    private lateinit var textSnack: TextView

    private val viewModel: BeverageViewModel by viewModels()
    private lateinit var menuListRV: RecyclerView
    private lateinit var beverageAdapter: BeverageAdapter
    private lateinit var menuList: List<BeverageClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.fragment_beverage, container, false)

        buttonCoffee = view.findViewById(R.id.buttonCoffee)
        buttonDrink = view.findViewById(R.id.buttonDrink)
        buttonSnack = view.findViewById(R.id.buttonSnack)
        textCoffee = view.findViewById(R.id.textCoffee)
        textDrink = view.findViewById(R.id.textDrink)
        textSnack = view.findViewById(R.id.textSnack)

        buttonCoffee.setOnClickListener{ handleButtonClick(buttonCoffee, textCoffee)}
        buttonDrink.setOnClickListener{ handleButtonClick(buttonDrink, textDrink)}
        buttonSnack.setOnClickListener{ handleButtonClick(buttonSnack, textSnack)}

        menuListRV = view.findViewById(R.id.menuListRV)

        return view
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        beverageAdapter = BeverageAdapter(requireContext(), emptyList())

        val sharedPreferences = requireActivity().getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val StoreId = sharedPreferences.getString("StoreId", "")

        menuListRV.layoutManager = GridLayoutManager(requireContext(), 4)
        menuListRV.adapter = beverageAdapter

        viewModel.getMenuList(StoreId!!)

        lifecycleScope.launch {
            viewModel.menuList.observe(viewLifecycleOwner) { menulist ->
                beverageAdapter.updateData(menulist) // 데이터 업데이트
                menuList = menulist
            }
        }

    }

    private fun handleButtonClick(clickedButton: ConstraintLayout, clickedTextView: TextView) {
        // 모든 버튼의 배경색을 원래대로 되돌림
        resetButtonBackgrounds()

        // 선택된 버튼의 배경색을 변경
        clickedButton.setBackgroundColor(Color.WHITE)
        clickedTextView.setTextColor(Color.BLACK)
    }

    private fun resetButtonBackgrounds() {
        val defCol = Color.parseColor("#5E412F")
        val defFont = Color.WHITE

        buttonCoffee.setBackgroundColor(defCol)
        buttonDrink.setBackgroundColor(defCol)
        buttonSnack.setBackgroundColor(defCol)

        textCoffee.setTextColor(defFont)
        textDrink.setTextColor(defFont)
        textSnack.setTextColor(defFont)
    }
}