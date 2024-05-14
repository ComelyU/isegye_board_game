package com.accio.isegye.customer.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
public class CreateImageRequest {
    @NotBlank
    String sourceFile;
    @NotBlank
    String themeFile;
}
