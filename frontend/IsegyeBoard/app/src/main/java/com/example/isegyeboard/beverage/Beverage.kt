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
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.isegyeboard.R
import com.example.isegyeboard.beverage.cart.CartAdapter
import com.example.isegyeboard.beverage.cart.CartClass
import com.example.isegyeboard.beverage.cart.CartViewModel
import com.example.isegyeboard.databinding.FragmentBeverageBinding
import kotlinx.coroutines.launch

class Beverage : Fragment(), CartAdapter.OnItemClickListener {
    private lateinit var buttonCoffee: ConstraintLayout
    private lateinit var buttonDrink: ConstraintLayout
    private lateinit var buttonSnack: ConstraintLayout

    private lateinit var textCoffee: TextView
    private lateinit var textDrink: TextView
    private lateinit var textSnack: TextView

    private val beverageViewModel: BeverageViewModel by viewModels()
    private lateinit var beverageListRV: RecyclerView
    private lateinit var beverageAdapter: BeverageAdapter
    private lateinit var beverageList: List<BeverageClass>

    private val cartViewModel: CartViewModel by viewModels()
    private lateinit var cartListRV: RecyclerView
    private lateinit var cartAdapter: CartAdapter


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

        beverageListRV = view.findViewById(R.id.menuListRV)
        cartListRV = view.findViewById(R.id.cartRV)

        return view
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        val sharedPreferences = requireActivity().getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val storeId = sharedPreferences.getString("StoreId", "1")

        beverageAdapter = BeverageAdapter(requireContext(), emptyList())

        beverageListRV.layoutManager = GridLayoutManager(requireContext(), 4)
        beverageListRV.adapter = beverageAdapter

        beverageViewModel.getMenuList(storeId!!)

        cartAdapter = CartAdapter(requireContext(), emptyList(), this)
        cartListRV.layoutManager = LinearLayoutManager(requireContext())
        cartListRV.adapter = cartAdapter

        lifecycleScope.launch {
            beverageViewModel.menuList.observe(viewLifecycleOwner) { menulist ->
                beverageList = menulist
                beverageAdapter.updateData(beverageList)
                handleButtonClick(buttonCoffee, textCoffee)
                beverageAdapter.notifyDataSetChanged()
            }

            cartViewModel.cartItems.observe(viewLifecycleOwner) { cartItems ->
                cartAdapter.updateData(cartItems)
            }
        }
    }

    override fun onItemRemoved(cartItem: CartClass) {
        cartViewModel.removeCartItem(cartItem)
    }
    private fun handleButtonClick(clickedButton: ConstraintLayout, clickedTextView: TextView) {
        // 모든 버튼의 배경색을 원래대로 되돌림
        resetButtonBackgrounds()

        // 선택된 버튼의 배경색을 변경
        clickedButton.setBackgroundColor(Color.WHITE)
        clickedTextView.setTextColor(Color.BLACK)

        val filteredList = when (clickedButton.id) {
            R.id.buttonCoffee -> beverageList.filter { it.menuType == "C" } // 카테고리에 따라 필터링
            R.id.buttonDrink -> beverageList.filter { it.menuType == "D" }
            R.id.buttonSnack -> beverageList.filter { it.menuType == "F" }
            else -> beverageList
        }
//        println(filteredList)

        beverageAdapter.updateData(filteredList)
        beverageAdapter.notifyDataSetChanged()
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